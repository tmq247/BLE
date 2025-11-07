// BLE-M3.c — Trigger từ event12, giữ theo hoạt động event11 (Android 14)

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

/* ===== Tham số tinh chỉnh ===== */
#define CAMERA_BURST_ABS       0x700   // nhận diện "chụp ảnh" ở event12 (REL lớn)
#define E11_QUIET_MS           300     // event11 im bao lâu thì nhả
#define WAIT_E11_TIMEOUT_MS    2000    // thời gian tối đa chờ event11 sau khi có event12
#define POLL_MS                40      // chu kỳ poll

static inline long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

/* ===== GRAB theo tên ===== */
static int open_by_name(const char *substr){
  char path[64], name[256];
  for(int i=0;i<64;i++){
    snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if(fd<0) continue;
    if(ioctl(fd,EVIOCGNAME(sizeof(name)),name)>=0 && strstr(name,substr)){
      ioctl(fd,EVIOCGRAB,1);
      fprintf(stderr,"[BLE-M3] Grabbed %s (%s)\n",path,name);
      return fd;
    }
    close(fd);
  }
  return -1;
}

/* ===== UINPUT: KEY_MEDIA (HEADSETHOOK) ===== */
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

/* ===== Trạng thái FSM ===== */
typedef enum { ST_IDLE=0, ST_AWAIT_E11, ST_HOLDING } State;
static State st = ST_IDLE;
static long long await_deadline = 0;     // hết hạn chờ event11
static long long last_e11_ms = 0;        // lần cuối có hoạt động event11
static int ptt_active = 0;

/* ===== Mouse (event12) — phát hiện burst để kích hoạt AWAIT_E11 ===== */
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
        // BẮT ĐẦU QUY TRÌNH: event12 báo hiệu -> đợi event11
        st = ST_AWAIT_E11;
        await_deadline = t + WAIT_E11_TIMEOUT_MS;
        // reset dấu mốc e11 để khi có e11 đầu tiên sẽ cập nhật
        last_e11_ms = 0;
        // không phát PTT ở đây; chỉ phát khi có event11 thật sự
        fprintf(stderr,"[BLE-M3] event12 burst -> AWAIT_E11\n");
      }
      reset_mouse();
      return;
    }
  }
}

/* ===== Consumer (event11) — dùng làm “nhịp giữ/nhả” ===== */
static void on_consumer_event(struct input_event *e){
  long long t = now_ms();

  // MỌI hoạt động từ event11 đều được coi là "đang còn hoạt động"
  if (e->type==EV_KEY || e->type==EV_MSC || (e->type==EV_SYN && e->code==SYN_REPORT)){
    last_e11_ms = t;
    if (st == ST_AWAIT_E11){
      // lần đầu thấy event11 kể từ khi AWAIT_E11 -> bắt đầu giữ
      if (!ptt_active){ ptt_down(); ptt_active=1; }
      st = ST_HOLDING;
      fprintf(stderr,"[BLE-M3] event11 activity -> HOLDING\n");
    }
  }

  // Không cần xử lý chi tiết 1/2/0 nữa; im lặng mới là tín hiệu nhả
}

/* ===== MAIN ===== */
int main(int argc,char**argv){
  signal(SIGINT, SIG_DFL);
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

    // Timeout khi đang chờ event11 mà không đến
    if (st == ST_AWAIT_E11 && t >= await_deadline){
      st = ST_IDLE;
      fprintf(stderr,"[BLE-M3] timeout AWAIT_E11 -> IDLE\n");
    }

    // Khi đang HOLDING: nhả nếu event11 im quá lâu
    if (st == ST_HOLDING){
      if (last_e11_ms>0 && (t - last_e11_ms) > E11_QUIET_MS){
        if (ptt_active){ ptt_up(); ptt_active=0; }
        st = ST_IDLE;
        fprintf(stderr,"[BLE-M3] event11 quiet -> RELEASE & IDLE\n");
      }
    }

    if(n<=0) continue;

    for(int i=0;i<2;i++){
      if(pfds[i].fd<0) continue;
      if(pfds[i].revents & (POLLERR|POLLHUP|POLLNVAL)){
        // thiết bị rớt -> nhả an toàn
        if(ptt_active){ ptt_up(); ptt_active=0; }
        st = ST_IDLE;
        continue;
      }
      if(!(pfds[i].revents & POLLIN)) continue;

      ssize_t r=read(pfds[i].fd,&ev,sizeof(ev));
      if(r!=sizeof(ev)) continue;

      if(pfds[i].fd==fd_mouse)  on_mouse_event(&ev);
      else if(pfds[i].fd==fd_cons) on_consumer_event(&ev);
    }
  }

  // Không bao giờ tới đây, nhưng để sạch sẽ:
  if(ptt_active){ ptt_up(); }
  if(fd_cons>=0){ ioctl(fd_cons,EVIOCGRAB,0); close(fd_cons); }
  if(fd_mouse>=0){ ioctl(fd_mouse,EVIOCGRAB,0); close(fd_mouse); }
  if(ufd>=0){ ioctl(ufd,UI_DEV_DESTROY); close(ufd); }
  return 0;
}
