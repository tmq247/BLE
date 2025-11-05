// blem3_learn_adb.c — Liệt kê thiết bị (scan /dev/input và fallback getevent -S),
// học chuỗi hành động BLE-M3, xác nhận & gán 1 key (F1..F12). Chạy tốt qua adb shell.
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
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define MAX_DEV 64
#define HOLD_MS     350
#define GAP_MS      120
#define DEADZONE    2
#define COOLDOWN_MS 120

typedef struct {
  char  name[256];
  char  path[64];
} Dev;

static Dev devs[MAX_DEV];
static int ndev = 0;

static long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}
static inline void shf(const char *fmt, ...) {
  char cmd[256]; va_list ap; va_start(ap, fmt);
  vsnprintf(cmd, sizeof(cmd), fmt, ap); va_end(ap);
  system(cmd);
}

/* ---------- Liệt kê: scan /dev/input ---------- */
static void scan_by_dir(void){
  ndev = 0;
  for (int i=0;i<64;i++){
    char path[64], name[256]; snprintf(path,sizeof(path),"/dev/input/event%d",i);
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) continue;
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) { close(fd); continue; }
    snprintf(devs[ndev].name, sizeof(devs[ndev].name), "%s", name);
    snprintf(devs[ndev].path, sizeof(devs[ndev].path), "%s", path);
    ndev++;
    close(fd);
    if (ndev >= MAX_DEV) break;
  }
}

/* ---------- Fallback: parse `getevent -S` ---------- */
static void trim(char *s){
  size_t n=strlen(s);
  while (n && (s[n-1]=='\n'||s[n-1]=='\r'||s[n-1]==' '||s[n-1]=='\t')) s[--n]=0;
}
static void scan_by_getevent(void){
  ndev = 0;
  FILE *fp = popen("/system/bin/getevent -S 2>/dev/null", "r");
  if (!fp) return;
  char line[512], last_name[256]="";
  while (fgets(line,sizeof(line),fp)){
    trim(line);
    // Examples from `getevent -S`:
    // add device 1: /dev/input/event12
    //   name:     "BLE-M3 Mouse"
    if (strncmp(line, "add device", 10)==0){
      char path[128]="";
      char *p=strstr(line, "/dev/input/event");
      if (p){
        strncpy(path, p, sizeof(path)-1);
        path[sizeof(path)-1]=0;
        // chờ dòng name ngay sau
        if (fgets(line,sizeof(line),fp)){
          trim(line);
          char *q=strstr(line,"name:");
          if (q){
            q=strchr(line,'"'); // mở quote
            if (q){
              q++;
              char *r=strchr(q,'"');
              if (r){ *r=0; snprintf(last_name,sizeof(last_name),"%s",q); }
            }
          }
        }
        if (last_name[0]){
          snprintf(devs[ndev].name,sizeof(devs[ndev].name),"%s",last_name);
          snprintf(devs[ndev].path,sizeof(devs[ndev].path),"%s",path);
          if (++ndev >= MAX_DEV) break;
          last_name[0]=0;
        }
      }
    }
  }
  pclose(fp);
}

/* ---------- Mở & GRAB ---------- */
static int open_grab_path(const char *path){
  int fd = open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return -1;
  ioctl(fd, EVIOCGRAB, 1);
  return fd;
}

/* ---------- Detect chuỗi hành động ---------- */
typedef struct {
  char action[32];     // UP_TAP, DOWN_HOLD, LEFT_TAP, RIGHT_TAP, CAMERA_TAP/HOLD, UNKNOWN
  char signature[64];  // khóa duy nhất để gán phím
} Detect;

static void detect_sequence(int *fds, int nfds, Detect *out){
  long long start = 0, last = 0, down_t0 = 0;
  int dx = 0, dy = 0;
  int saw_btn = 0, btn_down = 0, btn_up = 0;

  struct pollfd pf[8];
  for (int i=0;i<nfds;i++){ pf[i].fd=fds[i]; pf[i].events=POLLIN; pf[i].revents=0; }

  for(;;){
    int r = poll(pf, nfds, 3000);        // tối đa 3s chờ lần bấm đầu
    if (r <= 0) break;
    for (int i=0;i<nfds;i++){
      if (!(pf[i].revents & POLLIN)) continue;
      struct input_event ev; ssize_t n = read(pf[i].fd, &ev, sizeof(ev));
      if (n != sizeof(ev)) continue;
      if (start==0) start = now_ms();
      last = now_ms();

      if (ev.type == EV_KEY && ev.code == BTN_MOUSE){
        saw_btn = 1;
        if (ev.value == 1){ btn_down = 1; down_t0 = now_ms(); }
        if (ev.value == 0){ btn_up = 1; }
      } else if (ev.type == EV_REL){
        if (ev.code == REL_X){ if (abs(ev.value) >= DEADZONE) dx += ev.value; }
        if (ev.code == REL_Y){ if (abs(ev.value) >= DEADZONE) dy += ev.value; }
      }

      if (ev.type == EV_SYN && ev.code == SYN_REPORT){
        long long tnow = now_ms();
        if (tnow - last > GAP_MS) goto done;
      }
    }
    if (start && (now_ms() - last > GAP_MS)) break;
  }

done:
  memset(out, 0, sizeof(*out));
  if (saw_btn){
    long long dt = (btn_down && btn_up) ? (last - down_t0) : 0;
    if (dt >= HOLD_MS){
      snprintf(out->action, sizeof(out->action), "CAMERA_HOLD");
      snprintf(out->signature, sizeof(out->signature), "BTN_MOUSE:HOLD");
    } else {
      snprintf(out->action, sizeof(out->action), "CAMERA_TAP");
      snprintf(out->signature, sizeof(out->signature), "BTN_MOUSE:TAP");
    }
    return;
  }

  if (abs(dx) > abs(dy)){
    if (dx > 0){
      snprintf(out->action, sizeof(out->action), "RIGHT_TAP");
      snprintf(out->signature, sizeof(out->signature), "REL:X:+");
    } else if (dx < 0){
      snprintf(out->action, sizeof(out->action), "LEFT_TAP");
      snprintf(out->signature, sizeof(out->signature), "REL:X:-");
    } else {
      snprintf(out->action, sizeof(out->action), "UNKNOWN");
      snprintf(out->signature, sizeof(out->signature), "UNKNOWN");
    }
  } else if (abs(dy) > 0){
    long long dur = last - start;
    if (dy < 0){
      if (dur >= HOLD_MS){
        snprintf(out->action, sizeof(out->action), "UP_HOLD");
        snprintf(out->signature, sizeof(out->signature), "REL:Y:-:HOLD");
      } else {
        snprintf(out->action, sizeof(out->action), "UP_TAP");
        snprintf(out->signature, sizeof(out->signature), "REL:Y:-:TAP");
      }
    } else {
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
  int  keycode;  // 131..142 = F1..F12
  char name[16];
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

/* ---------- RUN loop ---------- */
static void run_loop(int *fds, int nfds){
  printf("\n[RUN] Nhấn phím trên remote — tôi in & phát key đã gán. (Ctrl+C để thoát)\n");
  long long last_emit = 0;
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

static void list_devices_print(void){
  printf("== Input devices ==\n");
  for (int i=0;i<ndev;i++){
    printf("[%d] %-26s %s\n", i, devs[i].name, devs[i].path);
  }
}

/* ---------- main ---------- */
int main(void){
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
  signal(SIGINT, SIG_DFL); signal(SIGTERM, SIG_DFL);

  // 1) Liệt kê: ưu tiên scan /dev/input, nếu không có → fallback getevent -S
  scan_by_dir();
  if (ndev == 0) {
    scan_by_getevent();
  }
  if (ndev == 0){
    fprintf(stderr,"Không liệt kê được thiết bị input. Hãy chạy từ adb shell.\n");
    return 1;
  }
  list_devices_print();

  printf("\nChọn thiết bị bằng chỉ số (cách nhau bởi dấu cách), rồi Enter.\n");
  printf("Gợi ý BLE-M3: chọn cả \"BLE-M3 Mouse\" và \"BLE-M3 Consumer Control\".\n> ");

  // 2) Lựa chọn
  int sel[4]; int nsel=0;
  char line[128]; if (!fgets(line,sizeof(line),stdin)) return 1;
  char *tok=strtok(line," \t\r\n");
  while(tok && nsel<4){ sel[nsel++]=atoi(tok); tok=strtok(NULL," \t\r\n"); }
  if (nsel==0){ printf("Không chọn thiết bị nào.\n"); return 1; }

  // 3) Mở & GRAB (theo path đã in)
  int fds[4]; int nfds=0;
  for (int i=0;i<nsel;i++){
    if (sel[i] < 0 || sel[i] >= ndev) continue;
    int fd = open_grab_path(devs[sel[i]].path);
    if (fd>=0){ fds[nfds++]=fd; printf("Grabbed: %s (%s)\n", devs[sel[i]].path, devs[sel[i]].name); }
    else printf("Không mở được: %s (%s)\n", devs[sel[i]].path, devs[sel[i]].name);
  }
  if (nfds==0){ printf("Không mở được thiết bị nào.\n"); return 1; }

  // 4) Vòng học/gán
  while (1){
    Detect d1={0}, d2={0};
    printf("\n[LEARN] Bấm 1 phím trên remote...\n");
    detect_sequence(fds, nfds, &d1);
    if (strcmp(d1.signature,"")==0){ printf("Không nhận được chuỗi.\n"); continue; }
    printf("→ Tôi thấy: %s (sig=%s)\n", d1.action, d1.signature);

    printf("Bấm lại phím đó để xác nhận...\n");
    detect_sequence(fds, nfds, &d2);
    if (strcmp(d1.signature, d2.signature)!=0){
      printf("Không khớp! Lần 2: %s (sig=%s)\n", d2.action, d2.signature);
      continue;
    }
    printf("OK, đã khớp: %s\n", d1.action);

    // Gán keycode
    printf("Nhập mã phím để gán (131..142=F1..F12), hoặc 's' để bỏ qua: ");
    if (!fgets(line,sizeof(line),stdin)) break;
    if (line[0]=='s' || line[0]=='S') continue;
    int code = atoi(line);
    if (code<=0){ printf("Mã không hợp lệ.\n"); continue; }

    // lưu map
    int idx=-1; for (int i=0;i<nmap;i++) if (!strcmp(maps[i].sig, d1.signature)) { idx=i; break; }
    if (idx<0 && nmap<MAX_MAP){ idx=nmap++; snprintf(maps[idx].sig,sizeof(maps[idx].sig),"%s",d1.signature); }
    if (idx<0){ printf("Hết slot map.\n"); continue; }
    maps[idx].keycode = code;
    snprintf(maps[idx].name, sizeof(maps[idx].name), "%s",
      (code>=131 && code<=142)? (const char*[]){"?","?","?","?","?","?","?","?","?","?","?","?"}[0] : "?");
    // tên đẹp
    switch(code){
      case 131: strcpy(maps[idx].name,"F1"); break;  case 132: strcpy(maps[idx].name,"F2"); break;
      case 133: strcpy(maps[idx].name,"F3"); break;  case 134: strcpy(maps[idx].name,"F4"); break;
      case 135: strcpy(maps[idx].name,"F5"); break;  case 136: strcpy(maps[idx].name,"F6"); break;
      case 137: strcpy(maps[idx].name,"F7"); break;  case 138: strcpy(maps[idx].name,"F8"); break;
      case 139: strcpy(maps[idx].name,"F9"); break;  case 140: strcpy(maps[idx].name,"F10"); break;
      case 141: strcpy(maps[idx].name,"F11"); break; case 142: strcpy(maps[idx].name,"F12"); break;
      default: snprintf(maps[idx].name,sizeof(maps[idx].name),"KEY_%d",code); break;
    }

    printf("→ GÁN: %s  =>  %s (%d)\n", d1.action, maps[idx].name, maps[idx].keycode);

    // Test ngay
    printf("Bấm lại phím vừa gán để kiểm tra...\n");
    Detect dt={0}; detect_sequence(fds, nfds, &dt);
    if (!strcmp(dt.signature,d1.signature)){
      printf("Khớp! Phát: %s (%d)\n", maps[idx].name, maps[idx].keycode);
      char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", maps[idx].keycode);
      shf("%s", cmd);
    } else {
      printf("Không khớp khi test (thấy %s / %s)\n", dt.action, dt.signature);
    }

    // Chọn vào RUN hay học tiếp
    printf("\nNhập 'r' để vào RUN, 'q' để thoát, phím khác để học tiếp: ");
    if (!fgets(line,sizeof(line),stdin)) break;
    if (line[0]=='r' || line[0]=='R'){ run_loop(fds,nfds); }
    else if (line[0]=='q' || line[0]=='Q') break;
  }

  for (int i=0;i<nfds;i++){ ioctl(fds[i],EVIOCGRAB,0); close(fds[i]); }
  return 0;
}
