// BLE-M3.c — BLE-M3 → F1..F12 (chống nhảy nhiều phím)
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

/*** F-keys map ***/
#define EMIT_UP_TAP         "input keyevent 131"   // F1
#define EMIT_UP_HOLD        "input keyevent 132"   // F2
#define EMIT_DOWN_TAP       "input keyevent 133"   // F3
#define EMIT_DOWN_HOLD      "input keyevent 134"   // F4
#define EMIT_LEFT_TAP       "input keyevent 135"   // F5
#define EMIT_LEFT_HOLD      "input keyevent 136"   // F6
#define EMIT_RIGHT_TAP      "input keyevent 137"   // F7
#define EMIT_RIGHT_HOLD     "input keyevent 138"   // F8
#define EMIT_CENTER_TAP     "input keyevent 139"   // F9  (nếu muốn dùng)
#define EMIT_CENTER_HOLD    "input keyevent 140"   // F10 (nếu muốn dùng)
#define EMIT_CAMERA_TAP     "input keyevent 141"   // F11
#define EMIT_CAMERA_HOLD    "input keyevent 142"   // F12

/*** Tùy chọn ***/
#define IGNORE_EVENT11_CAMERA   1   // 1: bỏ KEY_POWER từ event11 để tránh double
#define REL_DEADZONE            2   // bỏ nhiễu |value| < 2
#define HOLD_MS                 350 // >= hold
#define RELEASE_MS              120 // im > RELEASE_MS coi như thả
#define COOLDOWN_MS             120 // sau khi phát 1 phím, lặng trong COOLDOWN_MS
#define WINDOW_MS               20  // gom REL nhanh (nhỏ để phản hồi tốt)

/*** tiện ích ***/
static inline void sh(const char *cmd){ system(cmd); }
static long long now_ms(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000; }

static int open_by_name(const char *substr) {
  char path[64], name[256];
  for (int i=0;i<64;i++){
    snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if (fd<0) continue;
    if (ioctl(fd,EVIOCGNAME(sizeof(name)),name)>=0 && strstr(name,substr)){
      ioctl(fd,EVIOCGRAB,1);
      fprintf(stderr,"Grabbed %s (%s)\n",path,name);
      return fd;
    }
    close(fd);
  }
  return -1;
}

/*** trạng thái toàn cục ***/
static volatile int running=1;
static void stop(int s){ (void)s; running=0; }

/*** event11 (Consumer Control) — chỉ dùng khi không bỏ qua ***/
static long long e11_last_emit=0, e11_down_t0=0;
static void on_consumer_key(int code,int value){
#if IGNORE_EVENT11_CAMERA
  (void)code; (void)value;
  return;
#else
  long long t=now_ms();
  if (t - e11_last_emit < COOLDOWN_MS) return;   // cooldown
  if (code==KEY_POWER){
    if (value==1) e11_down_t0=t;
    else if (value==0){
      long long dt=t-e11_down_t0;
      if (dt>=HOLD_MS) sh(EMIT_CAMERA_HOLD);
      else             sh(EMIT_CAMERA_TAP);
      e11_last_emit=t;
    }
  }
#endif
}

/*** event12 (Mouse) — state machine ***/
typedef enum {IDLE, MODE_BTN, MODE_DIR} Mode;
typedef enum {AX_NONE, AX_X_POS, AX_X_NEG, AX_Y_POS, AX_Y_NEG} Axis;

static Mode  mode = IDLE;
static Axis  locked = AX_NONE;
static long long mode_t0=0, last_rel_ms=0, last_emit_ms=0;
static int acc = 0; // tích lũy theo trục đã khóa

static void reset_to_idle(void){
  mode = IDLE; locked = AX_NONE; acc = 0; mode_t0 = last_rel_ms = 0;
}

static void on_mouse_event(struct input_event *e){
  long long t = now_ms();

  // tôn trọng cooldown toàn cục
  if (last_emit_ms && (t - last_emit_ms) < COOLDOWN_MS) {
    // vẫn phải cập nhật BTN_MOUSE up để thoát MODE_BTN
    if (mode==MODE_BTN && e->type==EV_KEY && e->code==BTN_MOUSE && e->value==0)
      reset_to_idle();
    return;
  }

  // --- xử lý BTN_MOUSE (camera) ưu tiên tuyệt đối ---
  if (e->type==EV_KEY && e->code==BTN_MOUSE){
    if (e->value==1){                // down
      mode = MODE_BTN; mode_t0 = t; locked = AX_NONE; acc = 0;
    } else if (e->value==0 && mode==MODE_BTN){   // up
      long long dt = t - mode_t0;
      if (dt >= HOLD_MS) sh(EMIT_CAMERA_HOLD);
      else               sh(EMIT_CAMERA_TAP);
      last_emit_ms = t;
      reset_to_idle();
    }
    return;
  }

  // Nếu đang ở MODE_BTN → bỏ qua mọi REL/MSC
  if (mode==MODE_BTN) return;

  // --- xử lý REL để xác định hướng ---
  if (e->type==EV_REL){
    if (abs(e->value) < REL_DEADZONE) return;   // lọc nhiễu
    last_rel_ms = t;

    if (mode==IDLE){
      // khóa hướng ngay theo REL đầu tiên đủ lớn
      if (e->code==REL_X){
        locked = (e->value>0) ? AX_X_POS : AX_X_NEG;
      } else if (e->code==REL_Y){
        locked = (e->value>0) ? AX_Y_POS : AX_Y_NEG;
      }
      mode = MODE_DIR; mode_t0 = t; acc = 0;
    }

    if (mode==MODE_DIR){
      // chỉ tích lũy theo trục đã khóa
      if ((locked==AX_X_POS || locked==AX_X_NEG) && e->code==REL_X) {
        acc += e->value;
      } else if ((locked==AX_Y_POS || locked==AX_Y_NEG) && e->code==REL_Y) {
        acc += e->value;
      }
    }
    return;
  }

  // --- SYN_REPORT: kiểm tra nhả (im lặng) ---
  if (e->type==EV_SYN && e->code==SYN_REPORT && mode==MODE_DIR){
    if (t - last_rel_ms > RELEASE_MS && last_rel_ms!=0){
      // Kết thúc 1 lần nhấn theo hướng đã khóa → phát đúng 1 phím
      bool is_hold = (t - mode_t0) >= HOLD_MS;
      switch (locked){
        case AX_Y_NEG: sh(is_hold ? EMIT_UP_HOLD   : EMIT_UP_TAP);    break;
        case AX_Y_POS: sh(is_hold ? EMIT_DOWN_HOLD : EMIT_DOWN_TAP);  break;
        case AX_X_NEG: sh(is_hold ? EMIT_LEFT_HOLD : EMIT_LEFT_TAP);  break;
        case AX_X_POS: sh(is_hold ? EMIT_RIGHT_HOLD: EMIT_RIGHT_TAP); break;
        default: break;
      }
      last_emit_ms = t;
      reset_to_idle();
    }
  }
}

int main(void){
  signal(SIGINT,stop); signal(SIGTERM,stop);

  int fd_cons  = open_by_name("BLE-M3 Consumer Control");
  int fd_mouse = open_by_name("BLE-M3 Mouse");
  if (fd_cons<0 && fd_mouse<0){
    fprintf(stderr,"Không tìm thấy BLE-M3.\n"); return 1;
  }

  struct pollfd pfds[2]={{fd_cons,POLLIN,0},{fd_mouse,POLLIN,0}};
  struct input_event ev;

  while (running){
    int n=poll(pfds,2,300);
    if (n<=0) continue;
    for (int i=0;i<2;i++){
      if (pfds[i].fd<0 || !(pfds[i].revents&POLLIN)) continue;
      if (read(pfds[i].fd,&ev,sizeof(ev))!=sizeof(ev)) continue;

      if (pfds[i].fd==fd_cons  && ev.type==EV_KEY) on_consumer_key(ev.code,ev.value);
      else if (pfds[i].fd==fd_mouse)               on_mouse_event(&ev);
    }
  }

  if (fd_cons >=0){ ioctl(fd_cons,EVIOCGRAB,0); close(fd_cons); }
  if (fd_mouse>=0){ ioctl(fd_mouse,EVIOCGRAB,0); close(fd_mouse); }
  return 0;
}
