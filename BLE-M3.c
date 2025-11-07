// BLE-M3.c — PTT giữ theo phím BLE-M3 (Android). Giữ khi có event11; nhả khi event11 im.
// Hỗ trợ cả luồng "event12 -> đợi event11". Có log & ngưỡng im lặng tự thích nghi.

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
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

/* ==== Tham số ==== */
// Nhận diện "chụp ảnh" ở event12 (nếu dùng flow event12)
#define CAMERA_BURST_ABS        0x700

// Vùng an toàn nhả theo im lặng của event11
#define BASE_QUIET_MS           200     // bạn đang dùng 600ms
#define QUIET_FACTOR            15.0    // quiet_ms = max(BASE_QUIET_MS, avg_gap*factor)
#define EMA_ALPHA               0.30    // hệ số EMA cho avg_gap
#define POLL_MS                 40

// Debounce cho EV_KEY
#define KEY_DEBOUNCE_MS         20

static inline long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

/* ==== Mở & GRAB thiết bị theo tên ==== */
static int open_by_name(const char *substr){
  char path[64], name[256];
  for(int i=0;i<64;i++){
    snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if(fd<0) continue;
    if(ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0 && strstr(name, substr)){
      ioctl(fd, EVIOCGRAB, 1);
      fprintf(stderr, "[BLE-M3] Grabbed %s (%s)\n", path, name);
      return fd;
    }
    close(fd);
  }
  return -1;
}

/* ==== UINPUT: tạo KEY_MEDIA (HEADSETHOOK) ==== */
static int ufd=-1;
static int uinput_init(void){
  ufd=open("/dev/uinput", O_WRONLY|O_NONBLOCK);
  if(ufd<0){ perror("uinput"); return -1; }
  ioctl(ufd, UI_SET_EVBIT, EV_KEY);
  ioctl(ufd, UI_SET_KEYBIT, KEY_MEDIA);
  struct uinput_setup us={0};
  us.id.bustype=BUS_USB; us.id.vendor=0x1d6b; us.id.product=0x0104; us.id.version=1;
  snprintf(us.name, sizeof(us.name), "ble-m3-ptt");
  if(ioctl(ufd, UI_DEV_SETUP, &us) < 0) return -1;
  if(ioctl(ufd, UI_DEV_CREATE, 0) < 0)  return -1;
  usleep(100*1000);
  fprintf(stderr, "[BLE-M3] Created uinput PTT device\n");
  return 0;
}
static void emit_ev(int type,int code,int value){
  struct input_event ev={0}; struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  ev.type=type; ev.code=code; ev.value=value;
  ev.time.tv_sec=ts.tv_sec; ev.time.tv_usec=ts.tv_nsec/1000; write(ufd,&ev,sizeof(ev));
  struct input_event syn={0}; syn.type=EV_SYN; syn.code=SYN_REPORT;
  syn.time.tv_sec=ts.tv_sec; syn.time.tv_usec=ts.tv_nsec/1000; write(ufd,&syn,sizeof(syn));
}
static void ptt_down(void){ emit_ev(EV_KEY, KEY_MEDIA, 1); }
static void ptt_up(void){ emit_ev(EV_KEY, KEY_MEDIA, 0); }

/* ==== FSM ==== */
typedef enum { ST_IDLE=0, ST_AWAIT_E11, ST_HOLDING } State;
static State st = ST_IDLE;
static int   ptt_active = 0;

/* Log & nhịp event11 */
static long long e12_start_ms=0, first_e11_ms=0, last_e11_ms=0;
static double avg_gap_ms = 0.0;
static long long adaptive_quiet_ms = BASE_QUIET_MS;

/* Debounce EV_KEY */
static long long last_key_ms = 0;

/* ==== Mouse (event12) — kích hoạt AWAIT_E11 nếu cần ==== */
typedef struct { int btn_down; int max_dx, max_dy; } mouse_ctx_t;
static mouse_ctx_t M={0};
static void reset_mouse(void){ M.btn_down=0; M.max_dx=0; M.max_dy=0; }

static void on_mouse_event(struct input_event *e){
  long long t = now_ms();
  if(e->type==EV_REL){
    int v = e->value; if(v<0) v=-v;
    if(e->code==REL_X && v>M.max_dx) M.max_dx=v;
    else if(e->code==REL_Y && v>M.max_dy) M.max_dy=v;
    return;
  }
  if(e->type==EV_KEY && e->code==BTN_LEFT){
    if(e->value==1){ M.btn_down=1; M.max_dx=M.max_dy=0; return; }
    if(e->value==0 && M.btn_down){
      M.btn_down=0;
      int burst = (M.max_dx>=CAMERA_BURST_ABS) || (M.max_dy>=CAMERA_BURST_ABS);
      if(burst){
        st = ST_AWAIT_E11;
        e12_start_ms = t;
        first_e11_ms = 0; last_e11_ms=0; avg_gap_ms=0.0;
        fprintf(stderr,"[BLE-M3] e12 burst -> AWAIT_E11 (t=%lld)\n", e12_start_ms);
      }
      reset_mouse();
      return;
    }
  }
}

/* ==== Consumer (event11) — giữ/nhả theo hoạt động ==== */
static void maybe_start_holding_from_e11(long long t){
  // Nếu đang IDLE hoặc AWAIT_E11 thì thấy event11 -> bắt đầu giữ
  if (st != ST_HOLDING){
    if (!ptt_active){ ptt_down(); ptt_active=1; }
    if (st == ST_AWAIT_E11 && first_e11_ms==0){
      first_e11_ms = t;
      fprintf(stderr,"[BLE-M3] first e11 after e12: %lld ms\n", first_e11_ms - e12_start_ms);
    } else if (st == ST_IDLE){
      fprintf(stderr,"[BLE-M3] start HOLDING by e11 (no e12)\n");
    }
    st = ST_HOLDING;
    // reset thống kê gap
    avg_gap_ms = 0.0;
  }
}

static void on_consumer_event(struct input_event *e){
  long long t = now_ms();

  // Debounce EV_KEY
  if(e->type==EV_KEY){
    if(t - last_key_ms < KEY_DEBOUNCE_MS) return;
    last_key_ms = t;
  }

  // MỌI hoạt động event11 (KEY/MSC/SYN) đều tính là "còn hoạt động"
  if (e->type==EV_KEY || e->type==EV_MSC || (e->type==EV_SYN && e->code==SYN_REPORT)){
    // Nếu đang không giữ, có event11 -> vào giữ
    maybe_start_holding_from_e11(t);

    // Log & cập nhật avg gap
    if (last_e11_ms != 0){
      long long gap = t - last_e11_ms;
      const char *tt = (e->type==EV_KEY? "KEY" : (e->type==EV_MSC? "MSC" : "SYN"));
      fprintf(stderr,"[BLE-M3] e11 gap: %4lld ms  (%s code=0x%03x val=%d)\n",
              gap, tt, e->code, e->value);
      if (gap > 0){
        if (avg_gap_ms <= 0.1) avg_gap_ms = (double)gap;
        else avg_gap_ms = (1.0-EMA_ALPHA)*avg_gap_ms + EMA_ALPHA*(double)gap;
        long long new_quiet = (long long)(avg_gap_ms * QUIET_FACTOR);
        if (new_quiet < BASE_QUIET_MS) new_quiet = BASE_QUIET_MS;
        if (new_quiet != adaptive_quiet_ms){
          adaptive_quiet_ms = new_quiet;
          fprintf(stderr,"[BLE-M3] adaptive quiet -> %lld ms (avg_gap=%.1f)\n",
                  adaptive_quiet_ms, avg_gap_ms);
        }
      }
    } else {
      fprintf(stderr,"[BLE-M3] e11 activity seen (first)\n");
    }
    last_e11_ms = t;
  }

  // Ta không xét 1/2/0 nữa; chỉ nhả theo im lặng (xử lý ở vòng lặp)
}

/* ==== MAIN ==== */
int main(int argc,char**argv){
  signal(SIGINT,  SIG_DFL);
  signal(SIGTERM, SIG_DFL);

  if(uinput_init()!=0){ fprintf(stderr,"[BLE-M3] /dev/uinput lỗi\n"); return 1; }

  int fd_cons  = open_by_name("BLE-M3 Consumer Control");
  int fd_mouse = open_by_name("BLE-M3 Mouse");
  if(fd_cons<0 && fd_mouse<0){ fprintf(stderr,"[BLE-M3] Không thấy BLE-M3\n"); return 1; }

  struct pollfd pfds[2]={{fd_cons,POLLIN,0},{fd_mouse,POLLIN,0}};
  struct input_event ev;

  while(1){
    int n = poll(pfds, 2, POLL_MS);
    long long t = now_ms();

    // Khi đang giữ: nhả nếu event11 im quá ngưỡng quiet (adaptive)
    if (st == ST_HOLDING && last_e11_ms>0){
      long long quiet = t - last_e11_ms;
      long long th = adaptive_quiet_ms;               // ngưỡng đang dùng
      if (quiet > th){
        if (ptt_active){ ptt_up(); ptt_active=0; }
        fprintf(stderr,"[BLE-M3] RELEASE: e11 quiet %lld ms > %lld ms\n", quiet, th);
        st = ST_IDLE;
        e12_start_ms = first_e11_ms = last_e11_ms = 0; avg_gap_ms=0.0;
        adaptive_quiet_ms = BASE_QUIET_MS;
      }
    }

    if(n<=0) continue;

    for(int i=0;i<2;i++){
      if(pfds[i].fd<0) continue;
      if(pfds[i].revents & (POLLERR|POLLHUP|POLLNVAL)){
        if(ptt_active){ ptt_up(); ptt_active=0; }
        fprintf(stderr,"[BLE-M3] device hup/err -> IDLE\n");
        st = ST_IDLE;
        e12_start_ms = first_e11_ms = last_e11_ms = 0; avg_gap_ms=0.0;
        adaptive_quiet_ms = BASE_QUIET_MS;
        continue;
      }
      if(!(pfds[i].revents & POLLIN)) continue;

      ssize_t r=read(pfds[i].fd,&ev,sizeof(ev));
      if(r!=sizeof(ev)) continue;

      if(pfds[i].fd==fd_mouse)  on_mouse_event(&ev);
      else if(pfds[i].fd==fd_cons) on_consumer_event(&ev);
    }
  }

  // cleanup (thực tế hiếm khi tới đây)
  if(ptt_active){ ptt_up(); }
  if(fd_cons>=0){ ioctl(fd_cons,EVIOCGRAB,0); close(fd_cons); }
  if(fd_mouse>=0){ ioctl(fd_mouse,EVIOCGRAB,0); close(fd_mouse); }
  if(ufd>=0){ ioctl(ufd,UI_DEV_DESTROY); close(ufd); }
  return 0;
}
