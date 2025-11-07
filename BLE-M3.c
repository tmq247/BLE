// BLE-M3.c — event12 -> đợi event11; event11 giữ PTT; im lặng thì nhả.
// In log: t(event12), Δt(event12->event11 đầu), và gap giữa các event11 khi giữ.
// Android/Termux + /dev/uinput. Tested kiểu luồng đọc poll().

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

/* ======= Tham số tinh chỉnh ======= */
#define DEV_E12_NAME       "BLE-M3 Mouse"              // event12
#define DEV_E11_NAME       "BLE-M3 Consumer Control"   // event11
#define E11_QUIET_MS       600   // nhả nếu event11 im > ngưỡng này
#define POLL_MS            40    // chu kỳ poll
#define KEY_DEBOUNCE_MS    15    // chống dội EV_KEY

/* ======= Tiện ích thời gian ======= */
static inline long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

/* ======= Mở & GRAB thiết bị theo tên con ======= */
static int open_by_name(const char *substr, char *out_path, size_t out_sz){
  char path[64], name[256];
  for(int i=0;i<64;i++){
    snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if(fd<0) continue;
    if(ioctl(fd,EVIOCGNAME(sizeof(name)),name)>=0 && strstr(name,substr)){
      ioctl(fd,EVIOCGRAB,1);
      if(out_path && out_sz) snprintf(out_path,out_sz,"%s",path);
      fprintf(stderr,"[BLE-M3] Grabbed %s (%s)\n", path, name);
      return fd;
    }
    close(fd);
  }
  return -1;
}

/* ======= UINPUT: phát KEY_MEDIA làm PTT ======= */
static int ufd=-1;
static int uinput_init(void){
  ufd=open("/dev/uinput",O_WRONLY|O_NONBLOCK);
  if(ufd<0){ perror("uinput"); return -1; }
  ioctl(ufd,UI_SET_EVBIT,EV_KEY);
  ioctl(ufd,UI_SET_KEYBIT,KEY_MEDIA);

  struct uinput_setup us={0};
  us.id.bustype=BUS_USB; us.id.vendor=0x1d6b; us.id.product=0x0104; us.id.version=1;
  snprintf(us.name,sizeof(us.name),"ble-m3-ptt");
  if(ioctl(ufd,UI_DEV_SETUP,&us)<0) return -1;
  if(ioctl(ufd,UI_DEV_CREATE,0)<0)  return -1;
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
static void ptt_down(void){ emit_ev(EV_KEY,KEY_MEDIA,1); fprintf(stderr,"[BLE-M3] PTT DOWN\n"); }
static void ptt_up(void){ emit_ev(EV_KEY,KEY_MEDIA,0); fprintf(stderr,"[BLE-M3] PTT UP\n"); }

/* ======= FSM ======= */
typedef enum { ST_IDLE=0, ST_AWAIT_E11, ST_HOLDING } State;
static State st = ST_IDLE;
static int ptt_active = 0;

/* Mốc thời gian để log & nhả */
static long long e12_start_ms = 0;  // t(event12 bắt đầu)
static long long first_e11_ms = 0;  // t(event11 đầu sau e12)
static long long last_e11_ms  = 0;  // t(event11 gần nhất)
static long long last_key_ms  = 0;  // debounce EV_KEY

/* ======= event12: bắt đầu chuỗi chờ e11 ======= */
static void on_event12(struct input_event *e){
  // Kích hoạt khi có BTN_LEFT DOWN (tránh nhiễu từ move/rel lẻ tẻ)
  if(e->type==EV_KEY && e->code==BTN_LEFT && e->value==1){
    e12_start_ms = now_ms();
    first_e11_ms = 0;
    last_e11_ms  = 0;
    st = ST_AWAIT_E11;
    fprintf(stderr,"[BLE-M3] e12 start @ %lld ms -> AWAIT_E11\n", e12_start_ms);
  }
}

/* ======= event11: giữ/nhả + log gap ======= */
static void on_event11(struct input_event *e){
  long long t = now_ms();

  // chống dội KEY
  if(e->type==EV_KEY){
    if(t - last_key_ms < KEY_DEBOUNCE_MS) return;
    last_key_ms = t;
  }

  // Ta coi BẤT KỲ hoạt động e11 (KEY/MSC/SYN) là "vẫn đang có e11"
  if (e->type==EV_KEY || e->type==EV_MSC || (e->type==EV_SYN && e->code==SYN_REPORT)){
    // Nếu đang ở AWAIT_E11 -> log Δt và vào HOLDING
    if (st == ST_AWAIT_E11 && first_e11_ms == 0){
      first_e11_ms = t;
      fprintf(stderr,"[BLE-M3] first e11 after e12: %lld ms\n", first_e11_ms - e12_start_ms);
    }
    // Log gap giữa các lần e11
    if (last_e11_ms != 0){
      long long gap = t - last_e11_ms;
      const char *tt = (e->type==EV_KEY? "KEY" : (e->type==EV_MSC? "MSC" : "SYN"));
      fprintf(stderr,"[BLE-M3] e11 gap: %4lld ms  (%s code=0x%03x val=%d)\n",
              gap, tt, e->code, e->value);
    } else {
      fprintf(stderr,"[BLE-M3] e11 activity seen (first)\n");
    }
    last_e11_ms = t;

    // Nếu chưa hold thì bắt đầu giữ
    if (st != ST_HOLDING){
      if (!ptt_active){ ptt_down(); ptt_active=1; }
      st = ST_HOLDING;
    }
  }
}

/* ======= MAIN ======= */
int main(int argc,char**argv){
  signal(SIGINT, SIG_DFL);
  signal(SIGTERM, SIG_DFL);

  if(uinput_init()!=0){ fprintf(stderr,"[BLE-M3] /dev/uinput error\n"); return 1; }

  char p1[64], p2[64];
  int fd_e11 = open_by_name(DEV_E11_NAME, p1, sizeof(p1));
  int fd_e12 = open_by_name(DEV_E12_NAME, p2, sizeof(p2));
  if(fd_e11<0 || fd_e12<0){
    fprintf(stderr,"[BLE-M3] Không tìm thấy đủ thiết bị (%s / %s)\n", DEV_E11_NAME, DEV_E12_NAME);
    return 1;
  }

  struct pollfd pfds[2]={{fd_e11,POLLIN,0},{fd_e12,POLLIN,0}};
  struct input_event ev;

  while(1){
    int n = poll(pfds, 2, POLL_MS);
    long long t = now_ms();

    // Khi đang HOLDING: nhả nếu e11 im quá lâu
    if (st == ST_HOLDING && last_e11_ms>0){
      long long quiet = t - last_e11_ms;
      if (quiet > E11_QUIET_MS){
        if (ptt_active){ ptt_up(); ptt_active=0; }
        fprintf(stderr,"[BLE-M3] RELEASE: e11 quiet %lld ms > %d ms -> IDLE\n",
                quiet, E11_QUIET_MS);
        st = ST_IDLE;
        e12_start_ms = first_e11_ms = last_e11_ms = 0;
      }
    }

    if(n<=0) continue;

    for(int i=0;i<2;i++){
      if(!(pfds[i].revents & (POLLIN|POLLERR|POLLHUP|POLLNVAL))) continue;

      if(pfds[i].revents & (POLLERR|POLLHUP|POLLNVAL)){
        // thiết bị rớt -> nhả an toàn, về IDLE
        if(ptt_active){ ptt_up(); ptt_active=0; }
        st = ST_IDLE;
        e12_start_ms = first_e11_ms = last_e11_ms = 0;
        continue;
      }

      ssize_t r=read(pfds[i].fd,&ev,sizeof(ev));
      if(r!=sizeof(ev)) continue;

      if(pfds[i].fd==fd_e12) on_event12(&ev);
      else /* fd_e11 */      on_event11(&ev);
    }
  }

  // rarely reached
  if(ptt_active){ ptt_up(); }
  if(fd_e11>=0){ ioctl(fd_e11,EVIOCGRAB,0); close(fd_e11); }
  if(fd_e12>=0){ ioctl(fd_e12,EVIOCGRAB,0); close(fd_e12); }
  if(ufd>=0){ ioctl(ufd,UI_DEV_DESTROY); close(ufd); }
  return 0;
}
