// blem3_learn.c — Liệt kê thiết bị, học phím BLE-M3, gán F-key (131..142)
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

#define MAX_DEV 64
#define HOLD_MS     350          // >= hold
#define GAP_MS      120          // ngắt chuỗi event (khoảng lặng)
#define DEADZONE    2            // bỏ nhiễu REL nhỏ
#define COOLDOWN_MS 120

typedef struct {
  char  name[256];
  char  path[64];
  int   fd;
} Dev;

static Dev devs[MAX_DEV];
static int ndev = 0;

static inline long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}
static inline void shf(const char *fmt, ...) {
  char cmd[256]; va_list ap; va_start(ap, fmt);
  vsnprintf(cmd, sizeof(cmd), ap); va_end(ap);
  system(cmd);
}

/* ---------- Liệt kê thiết bị ---------- */
static void scan_devices(void){
  ndev = 0;
  for (int i=0;i<64;i++){
    char path[64], name[256]; snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) continue;
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) { close(fd); continue; }
    // Lưu
    snprintf(devs[ndev].name, sizeof(devs[ndev].name), "%s", name);
    snprintf(devs[ndev].path, sizeof(devs[ndev].path), "%s", path);
    devs[ndev].fd = fd; // tạm mở để show; sẽ đóng lại ngay sau in
    ndev++;
    close(fd);
    if (ndev >= MAX_DEV) break;
  }
}

/* ---------- Mở & GRAB theo chỉ số ---------- */
static int open_grab(int idx){
  if (idx < 0 || idx >= ndev) return -1;
  int fd = open(devs[idx].path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return -1;
  ioctl(fd, EVIOCGRAB, 1);
  return fd;
}

/* ---------- Phân loại chuỗi event thành “hành động” + “signature” ---------- */
typedef struct {
  char action[32];     // ví dụ: UP_TAP, DOWN_HOLD, LEFT_TAP, RIGHT_TAP, CAMERA_TAP, CAMERA_HOLD
  char signature[64];  // khóa học-một-lần (để khớp lần 2)
} Detect;

static void detect_sequence(int *fds, int nfds, Detect *out){
  // Thu thập 1 chuỗi event đến khi có khoảng lặng > GAP_MS
  long long start = 0, last = 0, down_t0 = 0;
  int dx = 0, dy = 0;
  int saw_key = 0, key_btnmouse = 0, key_down = 0, key_up = 0;

  struct pollfd pf[8];
  for (int i=0;i<nfds;i++){ pf[i].fd=fds[i]; pf[i].events=POLLIN; pf[i].revents=0; }

  for(;;){
    int r = poll(pf, nfds, 3000); // tối đa 3s chờ
    if (r <= 0) break; // hết thời gian — coi như không bấm
    for (int i=0;i<nfds;i++){
      if (!(pf[i].revents & POLLIN)) continue;
      struct input_event ev; ssize_t n = read(pf[i].fd, &ev, sizeof(ev));
      if (n != sizeof(ev)) continue;
      if (start==0) start = now_ms();
      last = now_ms();

      if (ev.type == EV_KEY){
        saw_key = 1;
        if (ev.code == BTN_MOUSE){
          key_btnmouse = 1;
          if (ev.value == 1){ key_down = 1; down_t0 = now_ms(); }
          if (ev.value == 0){ key_up = 1; }
        }
        // Có thể bổ sung KEY_POWER ở event11 nếu cần
      } else if (ev.type == EV_REL){
        if (ev.code == REL_X){ if (abs(ev.value) >= DEADZONE) dx += ev.value; }
        if (ev.code == REL_Y){ if (abs(ev.value) >= DEADZONE) dy += ev.value; }
      }
      if (ev.type == EV_SYN && ev.code == SYN_REPORT){
        // xem có khoảng lặng > GAP_MS không
        long long tnow = now_ms();
        if (tnow - last > GAP_MS) goto done;
      }
    }
    // khoảng lặng giữa 2 lần poll
    if (start && (now_ms() - last > GAP_MS)) break;
  }
done:
  // Quyết định hành động
  memset(out, 0, sizeof(*out));
  // 1) Nút giữa (camera) ưu tiên
  if (saw_key && key_btnmouse){
    long long dt = (key_down && key_up) ? (last - down_t0) : 0;
    if (dt >= HOLD_MS){
      snprintf(out->action, sizeof(out->action), "CAMERA_HOLD");
      snprintf(out->signature, sizeof(out->signature), "BTN_MOUSE:HOLD");
    } else {
      snprintf(out->action, sizeof(out->action), "CAMERA_TAP");
      snprintf(out->signature, sizeof(out->signature), "BTN_MOUSE:TAP");
    }
    return;
  }
  // 2) D-pad REL
  if (abs(dx) > abs(dy)){
    if (dx > 0){
      snprintf(out->action, sizeof(out->action), "RIGHT_TAP");
      snprintf(out->signature, sizeof(out->signature), "REL:X:+");
    } else {
      snprintf(out->action, sizeof(out->action), "LEFT_TAP");
      snprintf(out->signature, sizeof(out->signature), "REL:X:-");
    }
  } else if (abs(dx) < abs(dy) && abs(dy) > 0){
    if (dy < 0){
      // lên: dy âm
      long long dur = last - start;
      if (dur >= HOLD_MS){
        snprintf(out->action, sizeof(out->action), "UP_HOLD");
        snprintf(out->signature, sizeof(out->signature), "REL:Y:-:HOLD");
      } else {
        snprintf(out->action, sizeof(out->action), "UP_TAP");
        snprintf(out->signature, sizeof(out->signature), "REL:Y:-:TAP");
      }
    } else {
      long long dur = last - start;
      if (dur >= HOLD_MS){
        snprintf(out->action, sizeof(out->action), "DOWN_HOLD");
        snprintf(out->signature, sizeof(out->signature), "REL:Y:+:HOLD");
      } else {
        snprintf(out->action, sizeof(out->action), "DOWN_TAP");
        snprintf(out->signature, sizeof(out->signature), "REL:Y:+:TAP");
      }
    }
  } else {
    snprintf(out->action, sizeof(out->action), "UNKNOWN");
    snprintf(out->signature, sizeof(out->signature), "UNKNOWN");
  }
}

/* ---------- Bảng gán signature -> keycode ---------- */
typedef struct {
  char sig[64];
  int  keycode;  // 131..142 (F1..F12) hay phím khác
  char name[16]; // F1..F12 để in đẹp
} MapItem;

#define MAX_MAP 64
static MapItem maps[MAX_MAP];
static int nmap = 0;

static const char* keyname(int code){
  switch(code){
    case 131: return "F1"; case 132: return "F2"; case 133: return "F3";
    case 134: return "F4"; case 135: return "F5"; case 136: return "F6";
    case 137: return "F7"; case 138: return "F8"; case 139: return "F9";
    case 140: return "F10"; case 141: return "F11"; case 142: return "F12";
    default:  return "?";
  }
}
static int find_map(const char *sig){
  for (int i=0;i<nmap;i++) if (strcmp(maps[i].sig, sig)==0) return i;
  return -1;
}

/* ---------- Chế độ chạy: phát keyevent theo bảng gán ---------- */
static void run_loop(int *fds, int nfds){
  printf("\n[RUN] Nhấn phím trên remote — tôi sẽ in & phát key đã gán. (Ctrl+C để thoát)\n");
  long long last_emit = 0;
  struct pollfd pf[8]; for(int i=0;i<nfds;i++){pf[i].fd=fds[i];pf[i].events=POLLIN;pf[i].revents=0;}
  while (1){
    Detect d = {0};
    detect_sequence(fds, nfds, &d);
    if (strcmp(d.signature,"")==0) continue;
    int idx = find_map(d.signature);
    if (idx < 0){
      printf("→ %s (sig=%s) CHƯA GÁN\n", d.action, d.signature);
      continue;
    }
    if (now_ms()-last_emit < COOLDOWN_MS) continue;
    last_emit = now_ms();
    printf("→ %s  =>  %s (%d)\n", d.action, maps[idx].name, maps[idx].keycode);
    char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", maps[idx].keycode);
    shf("%s", cmd);
  }
}

/* ---------- main ---------- */
int main(void){
  signal(SIGINT, SIG_DFL); signal(SIGTERM, SIG_DFL);

  // 1) Liệt kê thiết bị
  scan_devices();
  if (ndev==0){ fprintf(stderr,"Không tìm thấy /dev/input/event*\n"); return 1; }
  printf("== Input devices ==\n");
  for (int i=0;i<ndev;i++){
    printf("[%d] %-22s  %s\n", i, devs[i].name, devs[i].path);
  }
  printf("\nChọn thiết bị bằng chỉ số (cách nhau bởi dấu cách), rồi Enter.\n");
  printf("Gợi ý cho BLE-M3: chọn cả \"%s\" và \"%s\" nếu có.\n",
         "BLE-M3 Mouse","BLE-M3 Consumer Control");
  printf("> ");

  // 2) Đọc lựa chọn
  int sel[4]; int nsel=0;
  char line[128]; if (!fgets(line,sizeof(line),stdin)) return 1;
  char *tok=strtok(line," \t\r\n");
  while(tok && nsel<4){ sel[nsel++]=atoi(tok); tok=strtok(NULL," \t\r\n"); }
  if (nsel==0){ printf("Không chọn thiết bị nào.\n"); return 1; }

  // 3) Mở & GRAB
  int fds[4]; int nfds=0;
  for (int i=0;i<nsel;i++){
    int fd = open_grab(sel[i]);
    if (fd>=0){ fds[nfds++]=fd; printf("Grabbed: %s (%s)\n", devs[sel[i]].path, devs[sel[i]].name); }
  }
  if (nfds==0){ printf("Không mở được thiết bị nào.\n"); return 1; }

  // 4) Vòng học gán
  while (1){
    Detect d1={0}, d2={0};
    printf("\n[LEARN] Bấm 1 phím trên remote...\n");
    detect_sequence(fds, nfds, &d1);
    if (strcmp(d1.signature,"")==0){ printf("Không nhận được chuỗi.\n"); continue; }
    printf("→ Tôi thấy: %s (sig=%s)\n", d1.action, d1.signature);

    printf("Bấm lại phím đó để xác nhận...\n");
    detect_sequence(fds, nfds, &d2);
    if (strcmp(d1.signature, d2.signature)!=0){
      printf("Không khớp! Tôi thấy lần 2 là: %s (sig=%s)\n", d2.action, d2.signature);
      continue;
    }
    printf("OK, đã khớp: %s\n", d1.action);

    // Nhập keycode
    printf("Nhập mã phím để gán (ví dụ 131..142 = F1..F12), hoặc 's' để bỏ qua: ");
    if (!fgets(line,sizeof(line),stdin)) break;
    if (line[0]=='s' || line[0]=='S') continue;
    int code = atoi(line);
    if (code<=0){ printf("Mã không hợp lệ.\n"); continue; }

    int idx = find_map(d1.signature);
    if (idx<0 && nmap<MAX_MAP){
      idx = nmap++;
      snprintf(maps[idx].sig, sizeof(maps[idx].sig), "%s", d1.signature);
    }
    maps[idx].keycode = code;
    snprintf(maps[idx].name, sizeof(maps[idx].name), "%s", keyname(code));
    printf("→ GÁN: %s  =>  %s (%d)\n", d1.action, maps[idx].name, maps[idx].keycode);

    // Thử ngay
    printf("Bấm lại phím vừa gán để kiểm tra (sẽ in & phát key)...\n");
    Detect dt={0}; detect_sequence(fds, nfds, &dt);
    if (strcmp(dt.signature,d1.signature)==0){
      printf("Khớp! Phát: %s (%d)\n", maps[idx].name, maps[idx].keycode);
      char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", maps[idx].keycode);
      shf("%s", cmd);
    } else {
      printf("Không khớp khi test (thấy %s / %s)\n", dt.action, dt.signature);
    }

    // Hỏi tiếp tục hay vào run
    printf("\nNhập 'r' để vào chế độ RUN, 'q' để thoát, phím khác để học tiếp: ");
    if (!fgets(line,sizeof(line),stdin)) break;
    if (line[0]=='r' || line[0]=='R'){ run_loop(fds,nfds); }
    else if (line[0]=='q' || line[0]=='Q') break;
  }

  for (int i=0;i<nfds;i++){ ioctl(fds[i],EVIOCGRAB,0); close(fds[i]); }
  return 0;
}
