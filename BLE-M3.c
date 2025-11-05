// BLE-M3.c — Grab BLE-M3 remote và phát F1–F12 theo hành động
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

/*** MÃ CỐ ĐỊNH → F1..F12 (Android keycodes 131..142) ***/
#define EMIT_UP_TAP         "input keyevent 131"   // F1
#define EMIT_UP_HOLD        "input keyevent 132"   // F2
#define EMIT_DOWN_TAP       "input keyevent 133"   // F3
#define EMIT_DOWN_HOLD      "input keyevent 134"   // F4
#define EMIT_LEFT_TAP       "input keyevent 135"   // F5
#define EMIT_LEFT_HOLD      "input keyevent 136"   // F6
#define EMIT_RIGHT_TAP      "input keyevent 137"   // F7
#define EMIT_RIGHT_HOLD     "input keyevent 138"   // F8
#define EMIT_CENTER_TAP     "input keyevent 139"   // F9
#define EMIT_CENTER_HOLD    "input keyevent 140"   // F10
#define EMIT_CAMERA_TAP     "input keyevent 141"   // F11
#define EMIT_CAMERA_HOLD    "input keyevent 142"   // F12

/*** THỜI GIAN & NGƯỠNG ***/
#define HOLD_MS    350
#define WINDOW_MS  100
#define THRESH_X   2
#define THRESH_Y   2
#define IDLE_MS    200
#define REPEAT_MS  120

static inline void sh(const char *cmd){ system(cmd); }
static long long now_ms(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000; }

/* ==== Tìm và GRAB thiết bị ==== */
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

static volatile int running=1;
static void stop(int s){(void)s;running=0;}

/*** ====== Xử lý event11: CAMERA ====== ***/
static long long cam_down_t0=0;
static void on_consumer_key(int code,int value){
  if (code==KEY_POWER){
    if (value==1) cam_down_t0=now_ms();
    else if (value==0){
      long long dt=now_ms()-cam_down_t0;
      if (dt>=HOLD_MS) sh(EMIT_CAMERA_HOLD);
      else             sh(EMIT_CAMERA_TAP);
    }
  }
}

/*** ====== Xử lý event12: D-PAD chuột ====== ***/
enum HoldMode {HM_NONE,HM_VOL_UP,HM_VOL_DOWN,HM_SCREEN_OFF};
static int dx_sum=0,dy_sum=0,middle_down=0;
static long long win_t0=0,last_activity=0,middle_t0=0,last_repeat_ms=0;
static enum HoldMode hold_mode=HM_NONE;

static inline void reset_window(void){dx_sum=dy_sum=0;win_t0=now_ms();}
static inline void end_hold_if_idle(long long now){
  if(hold_mode!=HM_NONE && now-last_activity>IDLE_MS){hold_mode=HM_NONE;last_repeat_ms=0;}
}

static void on_mouse_event(struct input_event *e){
  long long now=now_ms(); if(win_t0==0)win_t0=now;
  if(e->type==EV_REL||(e->type==EV_KEY&&e->code==BTN_MOUSE))last_activity=now;

  // Nút giữa
  if(e->type==EV_KEY&&e->code==BTN_MOUSE){
    if(e->value==1){middle_down=1;middle_t0=now;}
    else if(e->value==0){
      long long dt=now-middle_t0;
      middle_down=0;
      if(dt>=HOLD_MS)sh(EMIT_CENTER_HOLD);
      else           sh(EMIT_CENTER_TAP);
    }
    return;
  }

  // D-pad dịch chuyển
  if(e->type==EV_REL){
    if(e->code==REL_X)dx_sum+=e->value;
    if(e->code==REL_Y)dy_sum+=e->value;
    return;
  }

  if(e->type==EV_SYN&&e->code==SYN_REPORT){
    // lặp volume khi giữ
    if(hold_mode==HM_VOL_UP||hold_mode==HM_VOL_DOWN){
      if(now-last_repeat_ms>=REPEAT_MS){
        sh(hold_mode==HM_VOL_UP?EMIT_UP_HOLD:EMIT_DOWN_HOLD);
        last_repeat_ms=now;
      }
    }
    end_hold_if_idle(now);

    if(now-win_t0>WINDOW_MS){
      if(!middle_down&&hold_mode==HM_NONE){
        if(abs(dx_sum)>abs(dy_sum)&&abs(dx_sum)>=THRESH_X){
          if(dx_sum<0){sh(EMIT_LEFT_HOLD);hold_mode=HM_SCREEN_OFF;}
          else         sh(EMIT_RIGHT_TAP);
        }else if(abs(dy_sum)>=THRESH_Y){
          if(dy_sum<0){hold_mode=HM_VOL_UP;last_repeat_ms=0;sh(EMIT_UP_TAP);}
          else         {hold_mode=HM_VOL_DOWN;last_repeat_ms=0;sh(EMIT_DOWN_TAP);}
        }
      }
      reset_window();
    }
  }
}

int main(int argc,char**argv){
  signal(SIGINT,stop);signal(SIGTERM,stop);
  int fd_cons=-1,fd_mouse=-1;
  if(fd_cons<0)fd_cons=open_by_name("BLE-M3 Consumer Control");
  if(fd_mouse<0)fd_mouse=open_by_name("BLE-M3 Mouse");
  if(fd_cons<0&&fd_mouse<0){fprintf(stderr,"Không tìm thấy BLE-M3.\n");return 1;}

  struct pollfd pfds[2]={{fd_cons,POLLIN,0},{fd_mouse,POLLIN,0}};
  struct input_event ev;reset_window();

  while(running){
    int n=poll(pfds,2,300);if(n<=0)continue;
    for(int i=0;i<2;i++){
      if(pfds[i].fd<0||!(pfds[i].revents&POLLIN))continue;
      if(read(pfds[i].fd,&ev,sizeof(ev))!=sizeof(ev))continue;
      if(pfds[i].fd==fd_cons&&ev.type==EV_KEY)on_consumer_key(ev.code,ev.value);
      else if(pfds[i].fd==fd_mouse)on_mouse_event(&ev);
    }
  }

  if(fd_cons>=0){ioctl(fd_cons,EVIOCGRAB,0);close(fd_cons);}
  if(fd_mouse>=0){ioctl(fd_mouse,EVIOCGRAB,0);close(fd_mouse);}
  return 0;
}
