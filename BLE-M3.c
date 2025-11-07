// BLE-M3.c — Giữ mũi tên xuống = PTT hold (ổn định, watchdog động, Android 14)

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
#define CAMERA_BURST_ABS      0x700
#define COALESCE_MS           90
#define RELEASE_DELAY_MS      120
#define DEBOUNCE_MS           30
#define EMERGENCY_RELEASE_MS  2000   // tự nhả nếu mất tín hiệu lâu

static inline void sh(const char *cmd){ system(cmd); }
static long long now_ms(void){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

/* ===== GRAB thiết bị BLE-M3 ===== */
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

/* ===== UINPUT: tạo bàn phím ảo gửi KEY_MEDIA (HEADSETHOOK) ===== */
static int ufd = -1;
static int uinput_init(void){
    ufd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (ufd < 0){ perror("open /dev/uinput"); return -1; }
    ioctl(ufd, UI_SET_EVBIT, EV_KEY);
    ioctl(ufd, UI_SET_KEYBIT, KEY_MEDIA);

    struct uinput_setup us = {0};
    us.id.bustype = BUS_USB;
    us.id.vendor  = 0x1d6b;
    us.id.product = 0x0104;
    us.id.version = 1;
    snprintf(us.name, sizeof(us.name), "ble-m3-ptt");

    if (ioctl(ufd, UI_DEV_SETUP, &us) < 0) return -1;
    if (ioctl(ufd, UI_DEV_CREATE, 0) < 0) return -1;
    usleep(100*1000);
    fprintf(stderr,"[BLE-M3] Created uinput device for PTT\n");
    return 0;
}

static void emit_ev(int type,int code,int value){
    struct input_event ev={0};
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    ev.type=type; ev.code=code; ev.value=value;
    ev.time.tv_sec = ts.tv_sec; ev.time.tv_usec = ts.tv_nsec/1000;
    write(ufd,&ev,sizeof(ev));

    struct input_event syn={0};
    syn.type=EV_SYN; syn.code=SYN_REPORT; syn.value=0;
    syn.time.tv_sec = ts.tv_sec; syn.time.tv_usec = ts.tv_nsec/1000;
    write(ufd,&syn,sizeof(syn));
}

static void ptt_down(void){ emit_ev(EV_KEY, KEY_MEDIA, 1); }
static void ptt_up(void)  { emit_ev(EV_KEY, KEY_MEDIA, 0); }
static void ptt_tap(void) { ptt_down(); ptt_up(); }

/* ===== TRẠNG THÁI ===== */
static volatile int running=1;
static void stop(int s){(void)s;running=0;}

static int ptt_active = 0;
static long long last_vol_evt_ms = 0;
static long long pending_release_deadline = 0;
static int saw_repeat = 0;
static long long last_repeat_ms = 0;
static double avg_repeat_ms = 0.0;
static long long hold_start_ms = 0;

/* ===== CONSUMER (event11): Giữ mũi tên xuống = PTT hold ===== */
static void on_consumer_event(struct input_event *e){
    if (e->type != EV_KEY) return;

    long long t = now_ms();
    if (t - last_vol_evt_ms < DEBOUNCE_MS) return;
    last_vol_evt_ms = t;

    if (e->code == KEY_VOLUMEDOWN){
        if (e->value == 1){  // nhấn
            pending_release_deadline = 0;
            if (!ptt_active){
                ptt_down(); ptt_active = 1;
                saw_repeat = 0;
                last_repeat_ms = 0;
                avg_repeat_ms = 0.0;
                hold_start_ms = t;
            }
            return;
        }
        if (e->value == 2){  // repeat
            if (last_repeat_ms > 0){
                double dt = (double)(t - last_repeat_ms);
                if (avg_repeat_ms <= 1.0) avg_repeat_ms = dt;
                else avg_repeat_ms = 0.7*avg_repeat_ms + 0.3*dt;
            }
            last_repeat_ms = t;
            saw_repeat = 1;
            return;
        }
        if (e->value == 0){  // nhả
            pending_release_deadline = t + RELEASE_DELAY_MS;
            return;
        }
    }
}

/* ===== MOUSE (event12): “chụp ảnh” = PTT tap (nếu không đang giữ) ===== */
typedef struct { int btn_down; long long t_down; int max_dx, max_dy; } mouse_ctx_t;
static mouse_ctx_t M = {0};
static void reset_mouse(void){ M.max_dx=0; M.max_dy=0; M.btn_down=0; }

static void on_mouse_event(struct input_event *e){
    long long t = now_ms();
    if (e->type == EV_REL){
        int v = abs(e->value);
        if (e->code == REL_X && v > M.max_dx) M.max_dx = v;
        else if (e->code == REL_Y && v > M.max_dy) M.max_dy = v;
        return;
    }
    if (e->type == EV_KEY && e->code == BTN_LEFT){
        if (e->value == 1){
            M.btn_down = 1; reset_mouse();
        } else if (e->value == 0 && M.btn_down){
            M.btn_down = 0;
            int burst = (M.max_dx >= CAMERA_BURST_ABS) || (M.max_dy >= CAMERA_BURST_ABS);
            if (burst && !ptt_active) ptt_tap();
            reset_mouse();
        }
    }
}

/* ===== MAIN ===== */
int main(int argc,char**argv){
    signal(SIGINT,stop);
    signal(SIGTERM,stop);

    if (uinput_init()!=0){
        fprintf(stderr,"[BLE-M3] Không tạo được /dev/uinput\n");
        return 1;
    }

    int fd_cons  = open_by_name("BLE-M3 Consumer Control");
    int fd_mouse = open_by_name("BLE-M3 Mouse");
    if (fd_cons<0 && fd_mouse<0){
        fprintf(stderr,"[BLE-M3] Không thấy BLE-M3\n");
        return 1;
    }

    struct pollfd pfds[2]={{fd_cons,POLLIN,0},{fd_mouse,POLLIN,0}};
    struct input_event ev;

    while(running){
        int n = poll(pfds, 2, 40);
        long long t = now_ms();

        // --- WATCHDOG động ---
        if (ptt_active){
            if (pending_release_deadline>0 && t >= pending_release_deadline){
                ptt_up(); ptt_active=0; pending_release_deadline=0;
            }
            else if (saw_repeat){
                long long dyn = (long long)(3.0*avg_repeat_ms);
                if (dyn < 800) dyn = 800;
                if (last_repeat_ms>0 && (t - last_repeat_ms) > dyn){
                    ptt_up(); ptt_active=0;
                    pending_release_deadline=0; saw_repeat=0;
                }
            } else {
                if ((t - hold_start_ms) > EMERGENCY_RELEASE_MS){
                    ptt_up(); ptt_active=0;
                    pending_release_deadline=0;
                }
            }
        }

        if (n <= 0) continue;

        for (int i=0;i<2;i++){
            if (pfds[i].fd<0) continue;
            if (pfds[i].revents & (POLLERR|POLLHUP|POLLNVAL)){
                if (ptt_active){ ptt_up(); ptt_active=0; }
                continue;
            }
            if (!(pfds[i].revents & POLLIN)) continue;

            ssize_t r = read(pfds[i].fd,&ev,sizeof(ev));
            if (r != sizeof(ev)) continue;

            if (pfds[i].fd==fd_cons)  on_consumer_event(&ev);
            else if (pfds[i].fd==fd_mouse) on_mouse_event(&ev);
        }
    }

    if (fd_cons>=0){ ioctl(fd_cons,EVIOCGRAB,0); close(fd_cons); }
    if (fd_mouse>=0){ ioctl(fd_mouse,EVIOCGRAB,0); close(fd_mouse); }
    if (ufd>=0){ ioctl(ufd,UI_DEV_DESTROY); close(ufd); }
    return 0;
}
