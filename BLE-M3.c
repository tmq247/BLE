// BLE-M3.c — PTT hold bằng mũi tên xuống, chỉ nhả khi thấy value=0 (Android 14)

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

/* ===== Tham số ===== */
#define CAMERA_BURST_ABS        0x700  // nhận diện "chụp ảnh" ở event12
#define COALESCE_MS             90
#define DEBOUNCE_MS             25
#define CONFIRM_UP_MS           300    // thời gian "xác nhận nhả": nếu 1/2 xuất hiện -> hủy nhả
#define EMERGENCY_RELEASE_MS    6000   // mất kết nối thật sự mới tự nhả

static inline long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

/* ===== GRAB theo tên ===== */
static int open_by_name(const char *substr){
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

/* ===== UINPUT: tạo phím ảo phát KEY_MEDIA (HEADSETHOOK) ===== */
static int ufd=-1;
static int uinput_init(void){
  ufd=open("/dev/uinput",O_WRONLY|O_NONBLOCK);
  if(ufd<0){perror("uinput");return -1;}
  ioctl(ufd,UI_SET_EVBIT,EV_KEY);
  ioctl(ufd,UI_SET_KEYBIT,KEY_MEDIA);
  struct uinput_setup us={0};
  us.id.bustype=BUS_USB; us.id.vendor=0x1d6b; us.id.product=0x0104; us.id.version=1;
  snprintf(us.name,sizeof(us.name),"ble-m3-ptt");
  if(ioctl(ufd,UI_DEV_SETUP,&us)<0) return -1;
  if(ioctl(ufd,UI_DEV_CREATE,0)<0) return -1;
  usleep(100*1000);
  fprintf(stderr,"[BLE-M3] Created uinput PTT device\n");
  return 0;
}
static void emit_ev(int type,int code,int value){
  struct input_event ev={0}; struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  ev.type=type; ev.code=code; ev.value=value;
  ev.time.tv_sec=ts.tv_sec; ev.time.tv_usec=ts.tv_nsec/1000; write(ufd,&ev,sizeof(ev));
  struct input_event syn={0}; syn.type=EV_SYN; syn.code=SYN_REPORT;
  syn.time.tv_sec=ts.tv_sec; syn.time.tv_usec=ts.tv_nsec/1000; write(ufd,&syn,sizeof(syn));
}
static void ptt_down(void){ emit_ev(EV_KEY,KEY_MEDIA,1); }
static void ptt_up(void){ emit_ev(EV_KEY,KEY_MEDIA,0); }
static void ptt_tap(void){ ptt_down(); ptt_up(); }

/* ===== Trạng thái ===== */
static volatile int running=1; static void stop(int s){(void)s;running=0;}
static int ptt_active=0;
static long long last_evt_ms=0;              // chống nhiễu
static long long hold_start_ms=0;
static long long confirm_up_deadline=0;      // >0 nghĩa là đang chờ xác nhận nhả

/* ===== Consumer (event11) — GIỮ mũi tên xuống => PTT hold ===== */
static void on_consumer_event(struct input_event *e){
  if(e->type!=EV_KEY) return;
  long long t=now_ms();
  if(t-last_evt_ms<DEBOUNCE_MS) return;  // lọc rung rất nhanh
  last_evt_ms=t;

  if(e->code==KEY_VOLUMEDOWN){
    if(e->value==1){                      // DOWN
      // nếu đang đếm xác nhận nhả mà người dùng vẫn giữ/tiếp tục repeat -> hủy nhả
      confirm_up_deadline=0;
      if(!ptt_active){ ptt_down(); ptt_active=1; hold_start_ms=t; }
      return;
    }
    if(e->value==2){                      // REPEAT
      // chỉ dùng để hủy pending release nếu có
      if(confirm_up_deadline>0) confirm_up_deadline=0;
      return;
    }
    if(e->value==0){                      // UP
      // không nhả ngay -> chờ xác nhận trong CONFIRM_UP_MS
      confirm_up_deadline=t+CONFIRM_UP_MS;
      return;
    }
  }
}

/* ===== Mouse (event12) — “chụp ảnh” => PTT tap (khi không hold) ===== */
typedef struct {int btn_down; int max_dx, max_dy;} mouse_ctx_t;
static mouse_ctx_t M={0};
static void reset_mouse(void){ M.btn_down=0; M.max_dx=0; M.max_dy=0; }

static void on_mouse_event(struct input_event *e){
  if(e->type==EV_REL){
    int v=abs(e->value);
    if(e->code==REL_X && v>M.max_dx) M.max_dx=v;
    else if(e->code==REL_Y && v>M.max_dy) M.max_dy=v;
    return;
  }
  if(e->type==EV_KEY && e->code==BTN_LEFT){
    if(e->value==1){ M.btn_down=1; M.max_dx=M.max_dy=0; return; }
    if(e->value==0 && M.btn_down){
      int burst=(M.max_dx>=CAMERA_BURST_ABS)||(M.max_dy>=CAMERA_BURST_ABS);
      if(burst && !ptt_active) ptt_tap();
      reset_mouse(); return;
    }
  }
}

/* ===== MAIN ===== */
int main(int argc,char**argv){
  signal(SIGINT,stop); signal(SIGTERM,stop);
  if(uinput_init()!=0){ fprintf(stderr,"[BLE-M3] /dev/uinput lỗi\n"); return 1; }

  int fd_cons=open_by_name("BLE-M3 Consumer Control");
  int fd_mouse=open_by_name("BLE-M3 Mouse");
  if(fd_cons<0 && fd_mouse<0){ fprintf(stderr,"[BLE-M3] Không thấy BLE-M3\n"); return 1; }

  struct pollfd pfds[2]={{fd_cons,POLLIN,0},{fd_mouse,POLLIN,0}};
  struct input_event ev;

  while(running){
    int n=poll(pfds,2,40);
    long long t=now_ms();

    // 1) Xác nhận nhả: chỉ nhả khi hết CONFIRM_UP_MS mà KHÔNG có down/repeat nào chen vào
    if(ptt_active && confirm_up_deadline>0 && t>=confirm_up_deadline){
      ptt_up(); ptt_active=0; confirm_up_deadline=0;
    }
    // 2) Emergency: nếu giữ quá lâu mà không có sự kiện nào (mất kết nối) -> nhả
    if(ptt_active && (t-hold_start_ms)>EMERGENCY_RELEASE_MS && confirm_up_deadline==0){
      ptt_up(); ptt_active=0;
    }

    if(n<=0) continue;

    for(int i=0;i<2;i++){
      if(pfds[i].fd<0) continue;
      if(pfds[i].revents & (POLLERR|POLLHUP|POLLNVAL)){
        if(ptt_active){ ptt_up(); ptt_active=0; confirm_up_deadline=0; }
        continue;
      }
      if(!(pfds[i].revents & POLLIN)) continue;

      ssize_t r=read(pfds[i].fd,&ev,sizeof(ev));
      if(r!=sizeof(ev)) continue;

      if(pfds[i].fd==fd_cons) on_consumer_event(&ev);
      else if(pfds[i].fd==fd_mouse) on_mouse_event(&ev);
    }
  }

  if(fd_cons>=0){ ioctl(fd_cons,EVIOCGRAB,0); close(fd_cons); }
  if(fd_mouse>=0){ ioctl(fd_mouse,EVIOCGRAB,0); close(fd_mouse); }
  if(ufd>=0){ ioctl(ufd,UI_DEV_DESTROY); close(ufd); }
  return 0;
}
