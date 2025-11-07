// BLE-M3.c — Map GIỮ mũi tên xuống -> PTT hold (Android 14)

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

/*================== CẤU HÌNH PHÁT PTT ==================*/
// Zello thường map tốt với KEYCODE_HEADSETHOOK (79)
#define EMIT_PTT_TAP   "input keyevent 79"
// Nếu bạn có tool phát key down/up thật (uinput) thì thay 2 dòng dưới.
// Với 'input keyevent' Android không tách được down/up nhưng Zello vẫn nhận tốt
// nếu ta gọi DOWN ở nhấn và UP ở nhả (vẫn dùng cùng lệnh để đơn giản).
#define EMIT_PTT_DOWN  "input keyevent 79"
#define EMIT_PTT_UP    "input keyevent 79"

// Bật chế độ giữ-để-nói
#define HOLD_TO_TALK 1

/*================== THAM SỐ NHẬN DIỆN ==================*/
#define CLICK_MAX_MS       350
#define CAMERA_BURST_ABS   0x700
#define DEBOUNCE_MS        80
#define COALESCE_MS        90

static inline void sh(const char *cmd){ system(cmd); }
static long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

/*================== MỞ & GRAB THIẾT BỊ ==================*/
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

static volatile int running=1;
static void stop(int s){(void)s;running=0;}

/*================== CONSUMER (event11) ==================*
 * Map GIỮ mũi tên xuống (KEY_VOLUMEDOWN) -> PTT hold.
 * Bỏ qua value=2 (autorepeat). Không đụng tới VOLUMEUP.
 */
static long long last_consumer_ms=0;
static int ptt_active = 0;

static void on_consumer_event(struct input_event *e){
  if (e->type != EV_KEY) return;

  // Chỉ xử lý KEY_VOLUMEDOWN (0x72)
  if (e->code == KEY_VOLUMEDOWN){
    if (e->value == 1) {            // nhấn
      if (!ptt_active){
        sh(EMIT_PTT_DOWN);
        ptt_active = 1;
      }
      return;
    }
    if (e->value == 0) {            // nhả
      if (ptt_active){
        sh(EMIT_PTT_UP);
        ptt_active = 0;
      }
      return;
    }
    if (e->value == 2) {            // autorepeat -> bỏ qua
      return;
    }
  }
  // Các phím consumer khác: bỏ qua để không ảnh hưởng âm lượng, v.v.
}

/*================== MOUSE (event12) ==================*
 * Nhận diện “chụp ảnh” bằng burst REL rất lớn -> PTT tap (tùy chọn).
 */
typedef struct {
  int btn_down;
  long long t_down;
  int rel_seen;
  int max_abs_dx;
  int max_abs_dy;
  long long last_emit;
} mouse_ctx_t;

static mouse_ctx_t M = {0};

static void reset_mouse_window(long long t){
  M.rel_seen = 0;
  M.max_abs_dx = 0;
  M.max_abs_dy = 0;
  M.last_emit = t;
}

static void on_mouse_event(struct input_event *e){
  long long t = now_ms();

  if (e->type == EV_REL){
    if (e->code == REL_X){
      int v = e->value; if (v<0) v = -v;
      if (v > M.max_abs_dx) M.max_abs_dx = v;
      M.rel_seen = 1;
    } else if (e->code == REL_Y){
      int v = e->value; if (v<0) v = -v;
      if (v > M.max_abs_dy) M.max_abs_dy = v;
      M.rel_seen = 1;
    }
    return;
  }

  if (e->type == EV_KEY && e->code == BTN_LEFT){
    if (e->value == 1){
      M.btn_down = 1;
      M.t_down = t;
      reset_mouse_window(t);
      return;
    } else if (e->value == 0 && M.btn_down){
      M.btn_down = 0;

      // Có burst lớn (|REL| >= 0x700) -> coi là nút "chụp ảnh" đặc trưng BLE-M3
      int burst = (M.max_abs_dx >= CAMERA_BURST_ABS) || (M.max_abs_dy >= CAMERA_BURST_ABS);
      if (burst){
        // Nếu vẫn muốn dùng click "chụp ảnh" để PTT tap, giữ lại dòng dưới:
        sh(EMIT_PTT_TAP);
      }
      reset_mouse_window(t);
      return;
    }
  }

  if (e->type == EV_SYN && e->code == SYN_REPORT){
    if (t - M.last_emit > COALESCE_MS){
      M.last_emit = t;
    }
  }
}

/*================== MAIN ==================*/
int main(int argc,char**argv){
  signal(SIGINT,stop); signal(SIGTERM,stop);

  int fd_cons  = open_by_name("BLE-M3 Consumer Control");
  int fd_mouse = open_by_name("BLE-M3 Mouse");
  if (fd_cons<0 && fd_mouse<0){
    fprintf(stderr,"[BLE-M3] Không tìm thấy thiết bị BLE-M3.\n");
    return 1;
  }

  struct pollfd pfds[2] = {
    { fd_cons , POLLIN, 0 },
    { fd_mouse, POLLIN, 0 }
  };

  struct input_event ev;
  while (running){
    int n = poll(pfds, 2, 300);
    if (n <= 0) continue;

    for (int i=0;i<2;i++){
      if (pfds[i].fd < 0) continue;
      if (!(pfds[i].revents & POLLIN)) continue;
      ssize_t r = read(pfds[i].fd, &ev, sizeof(ev));
      if (r != sizeof(ev)) continue;

      if (pfds[i].fd == fd_cons){
        on_consumer_event(&ev);
      } else if (pfds[i].fd == fd_mouse){
        on_mouse_event(&ev);
      }
    }
  }

  if (fd_cons  >= 0){ ioctl(fd_cons , EVIOCGRAB, 0); close(fd_cons ); }
  if (fd_mouse >= 0){ ioctl(fd_mouse, EVIOCGRAB, 0); close(fd_mouse); }
  return 0;
}
