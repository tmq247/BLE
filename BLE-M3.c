// BLE-M3.c — Bắt đầu theo event11; nhả khi event11 im 500ms.
// Log: thời điểm bắt đầu (e11 first) + gap giữa các lần event11.
// PTT tùy chỉnh phím qua biến PTT_KEY.

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

/* ======= TÙY CHỈNH ======= */
#define DEV_E11_NAME       "BLE-M3 Consumer Control"
#define E11_QUIET_MS       500     // nhả nếu event11 im > 500 ms
#define POLL_MS            40      // chu kỳ poll
#define KEY_DEBOUNCE_MS    15      // chống dội EV_KEY
#define PTT_KEY            KEY_MEDIA   // 🔧 Phím PTT (HEADSETHOOK)
/*
  Một số phím gợi ý khác:
    KEY_CAMERA       // phím chụp ảnh
    KEY_F1           // phím F1
    KEY_PROG1        // phím lập trình (ít dùng)
    KEY_PLAYPAUSE    // multimedia play/pause
    KEY_MICMUTE      // tắt mic
*/

/* ======= Tiện ích thời gian ======= */
static inline long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

/* ======= Mở & GRAB thiết bị ======= */
static int open_by_name(const char *substr){
  char path[64], name[256];
  for(int i=0;i<64;i++){
    snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if(fd<0) continue;
    if(ioctl(fd,EVIOCGNAME(sizeof(name)),name)>=0 && strstr(name,substr)){
      ioctl(fd,EVIOCGRAB,1);
      fprintf(stderr,"[BLE-M3] Grabbed %s (%s)\n", path, name);
      return fd;
    }
    close(fd);
  }
  return -1;
}

/* ======= UINPUT setup ======= */
static int ufd=-1;
static int uinput_init(void){
  ufd=open("/dev/uinput",O_WRONLY|O_NONBLOCK);
  if(ufd<0){ perror("uinput"); return -1; }

  ioctl(ufd,UI_SET_EVBIT,EV_KEY);
  ioctl(ufd,UI_SET_KEYBIT,PTT_KEY);

  struct uinput_setup us={0};
  us.id.bustype=BUS_USB; us.id.vendor=0x1d6b; us.id.product=0x0104; us.id.version=1;
  snprintf(us.name,sizeof(us.name),"ble-m3-ptt");
  if(ioctl(ufd,UI_DEV_SETUP,&us)<0) return -1;
  if(ioctl(ufd,UI_DEV_CREATE,0)<0)  return -1;
  usleep(100*1000);
  fprintf(stderr,"[BLE-M3] Created uinput device (PTT key=%d)\n", PTT_KEY);
  return 0;
}

static void emit_key(int code,int val){
  struct input_event ev={0}; struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  ev.type=EV_KEY; ev.code=code; ev.value=val;
  ev.time.tv_sec=ts.tv_sec; ev.time.tv_usec=ts.tv_nsec/1000; write(ufd,&ev,sizeof(ev));
  struct input_event syn={0}; syn.type=EV_SYN; syn.code=SYN_REPORT;
  syn.time.tv_sec=ts.tv_sec; syn.time.tv_usec=ts.tv_nsec/1000; write(ufd,&syn,sizeof(syn));
}
static void ptt_down(void){ emit_key(PTT_KEY,1); fprintf(stderr,"[BLE-M3] PTT DOWN (%d)\n", PTT_KEY); }
static void ptt_up(void){ emit_key(PTT_KEY,0); fprintf(stderr,"[BLE-M3] PTT UP (%d)\n", PTT_KEY); }

/* ======= FSM & trạng thái ======= */
typedef enum { ST_IDLE=0, ST_HOLDING } State;
static State st = ST_IDLE;
static int ptt_active = 0;

static long long start_e11_ms = 0;
static long long last_e11_ms  = 0;
static long long last_key_ms  = 0;

/* ======= event11 handler ======= */
static void on_event11(struct input_event *e){
  long long t = now_ms();

  if(e->type==EV_KEY){
    if(t - last_key_ms < KEY_DEBOUNCE_MS) return;
    last_key_ms = t;
  }

  if (e->type==EV_KEY || e->type==EV_MSC || (e->type==EV_SYN && e->code==SYN_REPORT)){
    if (st == ST_IDLE){
      start_e11_ms = t;
      last_e11_ms  = 0;
      if (!ptt_active){ ptt_down(); ptt_active=1; }
      st = ST_HOLDING;
      fprintf(stderr,"[BLE-M3] HOLD start @ %lld ms (by e11)\n", start_e11_ms);
    }

    if (last_e11_ms != 0){
      long long gap = t - last_e11_ms;
      const char *tt = (e->type==EV_KEY? "KEY" : (e->type==EV_MSC? "MSC" : "SYN"));
      fprintf(stderr,"[BLE-M3] e11 gap: %4lld ms  (%s code=0x%03x val=%d)\n",
              gap, tt, e->code, e->value);
    } else {
      fprintf(stderr,"[BLE-M3] e11 activity seen (first)\n");
    }
    last_e11_ms = t;
  }
}

/* ======= MAIN ======= */
int main(int argc,char**argv){
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);

  if(uinput_init()!=0){ fprintf(stderr,"[BLE-M3] /dev/uinput error\n"); return 1; }

  int fd_e11 = open_by_name(DEV_E11_NAME);
  if(fd_e11<0){
    fprintf(stderr,"[BLE-M3] Không thấy thiết bị '%s'\n", DEV_E11_NAME);
    return 1;
  }

  struct pollfd pfd={fd_e11,POLLIN,0};
  struct input_event ev;

  while(1){
    int n = poll(&pfd, 1, POLL_MS);
    long long t = now_ms();

    if (st == ST_HOLDING && last_e11_ms>0){
      long long quiet = t - last_e11_ms;
      if (quiet > E11_QUIET_MS){
        if (ptt_active){ ptt_up(); ptt_active=0; }
        fprintf(stderr,"[BLE-M3] RELEASE: e11 quiet %lld ms > %d ms -> IDLE\n",
                quiet, E11_QUIET_MS);
        st = ST_IDLE;
        start_e11_ms = last_e11_ms = 0;
      }
    }

    if(n<=0) continue;
    if(pfd.revents & (POLLERR|POLLHUP|POLLNVAL)){
      if(ptt_active){ ptt_up(); ptt_active=0; }
      st = ST_IDLE;
      start_e11_ms = last_e11_ms = 0;
      continue;
    }
    if(!(pfd.revents & POLLIN)) continue;

    ssize_t r=read(pfd.fd,&ev,sizeof(ev));
    if(r==sizeof(ev)) on_event11(&ev);
  }

  if(ptt_active){ ptt_up(); }
  if(fd_e11>=0){ ioctl(fd_e11,EVIOCGRAB,0); close(fd_e11); }
  if(ufd>=0){ ioctl(ufd,UI_DEV_DESTROY); close(ufd); }
  return 0;
}
