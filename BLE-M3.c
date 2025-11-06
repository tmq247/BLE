// blem3_ptt_combo.c
// Bắt COMBO: (VOL_UP|VOL_DOWN từ "BLE-M3 Consumer Control") + (BTN_MOUSE click từ "BLE-M3 Mouse")
// kèm tọa độ chuột lượng tử hóa, rồi phát phím PTT cho Zello.
// Chạy tốt qua: adb shell /data/local/tmp/BLE-M3-PTT-COMBO

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

#define READ_TIMEOUT_MS 5000
#define COMBO_WINDOW_MS 600       // cửa sổ kết hợp vol & click
#define QPIX            16        // lượng tử hóa tọa độ tương đối
#define PTT_KEYCODE     79        // 25=VOLUME_DOWN (khuyên dùng cho Zello); 79=HEADSETHOOK

static long long now_ms(void) {
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void list_inputs(void){
  printf("== Input devices ==\n");
  for (int i=0;i<64;i++){
    char path[64]; snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd=open(path,O_RDONLY|O_CLOEXEC);
    if (fd<0) continue;
    char name[256];
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0)
      printf("[%d] %-30s %s\n", i, name, path);
    close(fd);
  }
}

typedef struct {
  int fd;
  char name[256];
  int evno;
} Dev;

static int open_dev(int idx, Dev *out){
  char path[64]; snprintf(path,sizeof(path),"/dev/input/event%d",idx);
  int fd=open(path,O_RDONLY|O_CLOEXEC);
  if(fd<0) return -1;
  out->fd=fd; out->evno=idx;
  ioctl(fd,EVIOCGNAME(sizeof(out->name)), out->name);
  return 0;
}

int main(void){
  setvbuf(stdout,NULL,_IONBF,0);
  setvbuf(stderr,NULL,_IONBF,0);

  list_inputs();

  // Người dùng chọn 2 device: Consumer Control (thường event11) + Mouse (thường event12)
  int idx_cons=-1, idx_mouse=-1;
  printf("\nNhập chỉ số Consumer Control (vd 11): ");
  scanf("%d", &idx_cons); while(getchar()!='\n' && !feof(stdin)); // clear line
  printf("Nhập chỉ số Mouse (vd 12): ");
  scanf("%d", &idx_mouse); while(getchar()!='\n' && !feof(stdin));

  Dev dcons={0}, dmouse={0};
  if (open_dev(idx_cons,&dcons)<0){ perror("open consumer"); return 1; }
  if (open_dev(idx_mouse,&dmouse)<0){ perror("open mouse"); return 1; }

  printf("\nConsumer: event%d (%s)\nMouse    : event%d (%s)\n",
         dcons.evno, dcons.name, dmouse.evno, dmouse.name);
  printf("COMBO cần: (VolumeUp|VolumeDown) + Click chuột trái trong %d ms → gửi keyevent %d\n",
         COMBO_WINDOW_MS, PTT_KEYCODE);
  printf("Nhấn Ctrl+C để thoát.\n");

  // Trạng thái theo dõi
  long long last_vol_t = 0;   // thời điểm thấy vol
  int last_vol_dir = 0;       // -1 = down, +1 = up
  long long last_click_t = 0; // thời điểm click hoàn tất
  int last_qx = 0, last_qy = 0;

  // trạng thái tích lũy chuột để lấy tọa độ tại thời điểm click
  int dx=0, dy=0;
  int saw_down=0;
  long long click_down_t0=0;

  // Poll hai thiết bị cùng lúc
  struct pollfd pfds[2];
  pfds[0].fd = dcons.fd;  pfds[0].events = POLLIN; pfds[0].revents=0;
  pfds[1].fd = dmouse.fd; pfds[1].events = POLLIN; pfds[1].revents=0;

  for(;;){
    int r = poll(pfds, 2, READ_TIMEOUT_MS);
    if (r <= 0) continue;

    // --- Consumer Control (Volume Up/Down) ---
    if (pfds[0].revents & POLLIN){
      struct input_event ev;
      if (read(dcons.fd, &ev, sizeof(ev)) == sizeof(ev)){
        // Có thể là MSC_SCAN 0x000c00e9/ea hoặc EV_KEY 115/114
        if (ev.type == EV_MSC && ev.code == MSC_SCAN){
          if (ev.value == 0x000c00e9){ last_vol_dir = +1; last_vol_t = now_ms(); } // VolUp
          else if (ev.value == 0x000c00ea){ last_vol_dir = -1; last_vol_t = now_ms(); } // VolDown
        } else if (ev.type == EV_KEY){
          if (ev.code == KEY_VOLUMEUP && ev.value == 1){ last_vol_dir = +1; last_vol_t = now_ms(); }
          else if (ev.code == KEY_VOLUMEDOWN && ev.value == 1){ last_vol_dir = -1; last_vol_t = now_ms(); }
        }
      }
    }

    // --- Mouse (click + tọa độ) ---
    if (pfds[1].revents & POLLIN){
      struct input_event ev;
      if (read(dmouse.fd, &ev, sizeof(ev)) == sizeof(ev)){
        if (ev.type == EV_REL){
          if (ev.code == REL_X) dx += ev.value;
          else if (ev.code == REL_Y) dy += ev.value;
        } else if (ev.type == EV_KEY && ev.code == BTN_MOUSE){
          if (ev.value == 1){ saw_down = 1; click_down_t0 = now_ms(); }
          else if (ev.value == 0 && saw_down){
            // kết thúc 1 lần click
            saw_down = 0;
            int qx = (dx>=0? (dx+QPIX/2):(dx-QPIX/2))/QPIX;
            int qy = (dy>=0? (dy+QPIX/2):(dy-QPIX/2))/QPIX;
            last_qx = qx; last_qy = qy; last_click_t = now_ms();
            dx = dy = 0;
            printf("CLICK  q=(%d,%d)  (dur=%lldms)\n", qx,qy, last_click_t-click_down_t0);
          }
        }
      }
    }

    // --- Kiểm tra tổ hợp COMBO ---
    if (last_vol_t && last_click_t &&
        llabs(last_click_t - last_vol_t) <= COMBO_WINDOW_MS){
      // In & phát PTT
      const char *vname = (last_vol_dir>0? "VOL_UP":"VOL_DOWN");
      printf("[COMBO] %s + CLICK q=(%d,%d)  → input keyevent %d\n",
             vname, last_qx, last_qy, PTT_KEYCODE);
      char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", PTT_KEYCODE);
      system(cmd);

      // reset để tránh lặp
      last_vol_t = 0; last_click_t = 0; last_vol_dir = 0;
    }
  }

  // never reached
  return 0;
}
