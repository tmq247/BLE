// BLE-M3.c — PTT hold mũi tên xuống, phát HEADSETHOOK bằng uinput (Android 14)

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/uinput.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/* ===== Tham số ===== */
#define CAMERA_BURST_ABS   0x700
#define COALESCE_MS        90
#define RELEASE_DELAY_MS   120     // trì hoãn nhả để chống dội
#define DEBOUNCE_MS        40

static inline void sh(const char *cmd){ system(cmd); }
static long long now_ms(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000; }

/* ===== Mở & GRAB thiết bị thật ===== */
static int open_by_name(const char *substr) {
  char path[64], name[256];
  for (int i=0;i<64;i++){
    snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if (fd<0) continue;
    if (ioctl(fd,EVIOCGNAME(sizeof(name)),name)>=0 && strstr(name,substr)){
      ioctl(fd,EVIOCGRAB,1);
      fprintf(stderr,"[BLE-M3] Grabbed %s (%s)\n",path,name);
      return fd;
    }
    close(fd);
  }
  return -1;
}

/* ===== UINPUT: tạo bàn phím ảo phát KEY_MEDIA (-> HEADSETHOOK) ===== */
static int ufd = -1;

static int uinput_init(void){
  ufd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
  if (ufd < 0){ perror("open /dev/uinput"); return -1; }

  if (ioctl(ufd, UI_SET_EVBIT, EV_KEY) < 0) return -1;
  if (ioctl(ufd, UI_SET_KEYBIT, KEY_MEDIA) < 0) return -1; // 226

  struct uinput_setup us = {0};
  us.id.bustype = BUS_USB;
  us.id.vendor  = 0x1d6b;  // dummy
  us.id.product = 0x0104;  // dummy
  us.id.version = 1;
  snprintf(us.name, sizeof(us.name), "ble-m3-ptt");

  if (ioctl(ufd, UI_DEV_SETUP, &us) < 0) return -1;
  if (ioctl(ufd, UI_DEV_CREATE) < 0) return -1;

  // nhỏ delay cho thiết bị sẵn sàng
  usleep(100*1000);
  fprintf(stderr,"[BLE-M3] Created uinput device for PTT\n");
  return 0;
}

static void uinput_emit(int type, int code, int value){
  struct input_event ev = {0};
  ev.type = type; ev.code = code; ev.value = value;
  clock_gettime(CLOCK_MONOTONIC, &ev.time);
  write(ufd, &ev, sizeof(ev));

  struct input_event syn = {0};
  syn.type = EV_SYN; syn.code = SYN_REPORT; syn.value = 0;
  clock_gettime(CLOCK_MONOTONIC, &syn.time);
  write(ufd, &syn, sizeof(syn));
}

static void ptt_down(void){ uinput_emit(EV_KEY, KEY_MEDIA, 1); }
static void ptt_up(void)  { uinput_emit(EV_KEY, KEY_MEDIA, 0); }
static void ptt_tap(void) { uinput_emit(EV_KEY, KEY_MEDIA, 1); uinput_emit(EV_KEY, KEY_MEDIA, 0); }

/* ===== Trạng thái PTT & debounce ===== */
static volatile int running=1;
static void stop(int s){(void)s;running=0;}

static int ptt_active = 0;
static long long last_vol_evt_ms = 0;
static long long pending_release_deadline = 0;

/* ===== Consumer (event11): GIỮ mũi tên xuống -> PTT hold ===== */
static void on_consumer_event(struct input_event *e){
  if (e->type != EV_KEY) return;

  long long t = now_ms();
  if (t - last_vol_evt_ms < DEBOUNCE_MS) return; // chống rung nhanh
  last_vol_evt_ms = t;

  if (e->code == KEY_VOLUMEDOWN){
    if (e->value == 1){           // nhấn
      pending_release_deadline = 0; // hủy nhả trễ nếu có
      if (!ptt_active){ ptt_down(); ptt_active = 1; }
      return;
    }
    if (e->value == 0){           // nhả
      // nhả trễ để tránh dội 0→1→0 rất nhanh
      pending_release_deadline = t + RELEASE_DELAY_MS;
      return;
    }
    // value==2 (autorepeat) -> bỏ qua
  }
}

/* ===== Mouse (event12): “chụp ảnh” -> PTT tap, NHƯNG KHÔNG khi đang hold ===== */
typedef struct {
  int btn_down;
  long long t_down;
  int max_abs_dx, max_abs_dy;
  long long last_emit;
} mouse_ctx_t;

static mouse_ctx_t M = {0};

static void reset_mouse(long long t){
  M.max_abs_dx = M.max_abs_dy = 0;
  M.last_emit = t;
}

static void on_mouse_event(struct input_event *e){
  long long t = now_ms();

  if (e->type == EV_REL){
    int v = e->value; if (v<0) v = -v;
    if (e->code == REL_X){ if (v > M.max_abs_dx) M.max_abs_dx = v; }
    else if (e->code == REL_Y){ if (v > M.max_abs_dy) M.max_abs_dy = v; }
    return;
  }

  if (e->type == EV_KEY && e->code == BTN_LEFT){
    if (e->value == 1){
      M.btn_down = 1; M.t_down = t; reset_mouse(t);
      return;
    } else if (e->value == 0 && M.btn_down){
      M.btn_down = 0;
      int burst = (M.max_abs_dx >= CAMERA_BURST_ABS) || (M.max_abs_dy >= CAMERA_BURST_ABS);
      if (burst && !ptt_active){
        // chỉ tap khi KHÔNG đang giữ PTT
        ptt_tap();
      }
      reset_mouse(t);
      return;
    }
  }

  if (e->type == EV_SYN && e->code == SYN_REPORT){
    if (t - M.last_emit > COALESCE_MS) M.last_emit = t;
  }
}

/* ===== MAIN ===== */
int main(int argc,char**argv){
  signal(SIGINT,stop); signal(SIGTERM,stop);

  if (uinput_init() != 0){
    fprintf(stderr,"[BLE-M3] Không tạo được uinput. Cần root & /dev/uinput.\n");
    return 1;
  }

  int fd_cons  = open_by_name("BLE-M3 Consumer Control");
  int fd_mouse = open_by_name("BLE-M3 Mouse");
  if (fd_cons<0 && fd_mouse<0){
    fprintf(stderr,"[BLE-M3] Không tìm thấy BLE-M3.\n");
    return 1;
  }

  struct pollfd pfds[2] = {
    { fd_cons , POLLIN, 0 },
    { fd_mouse, POLLIN, 0 }
  };

  struct input_event ev;
  while (running){
    // timeout ngắn để xử lý nhả trễ
    int n = poll(pfds, 2, 50);
    long long t = now_ms();

    // xử lý nhả trễ
    if (ptt_active && pending_release_deadline>0 && t >= pending_release_deadline){
      ptt_up(); ptt_active = 0; pending_release_deadline = 0;
    }

    if (n <= 0) continue;

    for (int i=0;i<2;i++){
      if (pfds[i].fd < 0) continue;
      if (!(pfds[i].revents & POLLIN)) continue;
      ssize_t r = read(pfds[i].fd, &ev, sizeof(ev));
      if (r != sizeof(ev)) continue;

      if (pfds[i].fd == fd_cons)  on_consumer_event(&ev);
      else if (pfds[i].fd == fd_mouse) on_mouse_event(&ev);
    }
  }

  if (fd_cons  >= 0){ ioctl(fd_cons , EVIOCGRAB, 0); close(fd_cons ); }
  if (fd_mouse >= 0){ ioctl(fd_mouse, EVIOCGRAB, 0); close(fd_mouse); }

  if (ufd >= 0){
    ioctl(ufd, UI_DEV_DESTROY);
    close(ufd);
  }
  return 0;
}
