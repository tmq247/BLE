// BLE-M3.c — Grab BLE-M3 remote và gán PTT cho “chụp ảnh” (Android 14)
// Biên dịch bằng NDK/clang như bạn đang dùng.

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
#define ANDROID_KEYCODE_PTT 79

// Cách phát PTT mặc định: TAP (đẩy 1 keyevent 79)
// Nếu bạn muốn “nhấn để nói, nhả để ngừng”, bật HOLD_TO_TALK = 1 và điền Sendevent cho thiết bị input ảo của bạn.
#define HOLD_TO_TALK 0

// Nếu HOLD_TO_TALK=1, điền lệnh gửi key down/up (ví dụ input-helper riêng của bạn)
#define EMIT_PTT_TAP   "input keyevent 79"
#define EMIT_PTT_DOWN  "input keyevent 79"   // đổi thành tool của bạn nếu có tách down/up
#define EMIT_PTT_UP    "input keyevent 79"   // đổi thành tool của bạn nếu có tách down/up

/*================== THAM SỐ NHẬN DIỆN ==================*/
#define CLICK_MAX_MS       350     // dưới mốc này xem như bấm ngắn
#define CAMERA_BURST_ABS   0x700   // biên độ REL_X/REL_Y lớn coi là “chụp ảnh” (khớp log ±0x7ff/0x801)
#define DEBOUNCE_MS        80      // tránh double-report
#define COALESCE_MS        90      // gom sự kiện trong cửa sổ ngắn

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
 * Chủ yếu để chặn volume nếu cần, và dự phòng camera nếu remote đẩy qua đây.
 */
static long long last_consumer_ms=0;

static void on_consumer_event(struct input_event *e){
  long long t = now_ms();
  if (t - last_consumer_ms < DEBOUNCE_MS) return;

  if (e->type==EV_KEY){
    // 0x72=KEY_VOLUMEDOWN(114), 0x73=KEY_VOLUMEUP(115) theo log của bạn
    if ((e->code==KEY_VOLUMEUP || e->code==KEY_VOLUMEDOWN) && e->value==1){
#if HOLD_TO_TALK
      sh(EMIT_PTT_DOWN);
#else
      sh(EMIT_PTT_TAP);
#endif
      last_consumer_ms = t;
      return;
    }

    // Nhiều BLE remote gửi "power/camera" như KEY_POWER hoặc KEY_CAMERA
    if ((e->code==KEY_POWER || e->code==KEY_CAMERA) && e->value==1){
#if HOLD_TO_TALK
      sh(EMIT_PTT_DOWN);
#else
      sh(EMIT_PTT_TAP);
#endif
      last_consumer_ms = t;
      return;
    }

#if HOLD_TO_TALK
    // Nhả phím (volume/camera) thì nhả PTT
    if ((e->code==KEY_VOLUMEUP || e->code==KEY_VOLUMEDOWN ||
         e->code==KEY_POWER     || e->code==KEY_CAMERA) && e->value==0){
      sh(EMIT_PTT_UP);
      last_consumer_ms = t;
      return;
    }
#endif
  }
}

/*================== MOUSE (event12) ==================*
 * Nhận diện “chụp ảnh” của BLE-M3:
 *  - Có BTN_LEFT (EV_KEY, BTN_LEFT)
 *  - Trong khoảng nhấn, xuất hiện REL_X/REL_Y có |giá trị| >= CAMERA_BURST_ABS
 *    (theo ảnh của bạn: fffff801 / 000007ff / fffffe65 ... là các xung lớn).
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
      // BTN_LEFT down
      M.btn_down = 1;
      M.t_down = t;
      reset_mouse_window(t);
      return;
    } else if (e->value == 0 && M.btn_down){
      // BTN_LEFT up → phân loại
      M.btn_down = 0;
      long long dt = t - M.t_down;

      // “Signature chụp ảnh”: có xung lớn trong cửa sổ bấm (|rel| >= CAMERA_BURST_ABS)
      int burst = (M.max_abs_dx >= CAMERA_BURST_ABS) || (M.max_abs_dy >= CAMERA_BURST_ABS);

      if (burst){
        // Đây chính là nút chụp ảnh của BLE-M3 (lần bấm 1/2 đều tạo burst)
#if HOLD_TO_TALK
        // Giữ thì DOWN lúc bắt đầu, UP khi nhả (ở đây thực hiện ở cạnh UP cho chắc)
        // Nếu cần DOWN ngay khi nhấn, chuyển sang phát DOWN ở nhánh value==1 bên trên.
        sh(EMIT_PTT_UP); // kết thúc nói
#else
        // PTT tap (phù hợp Zello chế độ “nhấn để nói”)
        sh(EMIT_PTT_TAP);
#endif
      } else {
        // Click chuột trái bình thường → bỏ qua (không đụng vào PTT)
        // Nếu muốn map riêng click chuột trái “thường”, thêm lệnh ở đây.
      }
      reset_mouse_window(t);
      return;
    }
  }

  // Gom sự kiện trong một SYN_REPORT, phòng spam (không cần làm gì ở đây)
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
