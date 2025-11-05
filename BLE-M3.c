// blem3_learn_adb.c — Học phím BLE-M3 qua adb shell: liệt kê thiết bị,
// chọn theo số, gom một chuỗi hành động -> 1 signature duy nhất và cho gán F-key.
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
#define MAX_SEL 8
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

/* ---------------- Liệt kê thiết bị ---------------- */
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
    if (strncmp(line, "add device", 10)==0){
      char path[128]="";
      char *p=strstr(line, "/dev/input/event");
      if (p){
        strncpy(path, p, sizeof(path)-1);
        path[sizeof(path)-1]=0;
        if (fgets(line,sizeof(line),fp)){
          trim(line);
          char *q=strstr(line,"name:");
          if (q){
            q=strchr(line,'"'); if (q){ q++; char *r=strchr(q,'"'); if (r){ *r=0; snprintf(last_name,sizeof(last_name),"%s",q); } }
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
static void list_devices_print(void){
  printf("== Input devices ==\n");
  for (int i=0;i<ndev;i++){
    printf("[%d] %-28s %s\n", i, devs[i].name, devs[i].path);
  }
}

/* ---------------- Mở & GRAB ---------------- */
typedef struct {
  int fd;
  char name[256];
  char path[64];
  int evno;   // số eventXX
  char tag[8]; // "E11" / "E12" ...
} SelFD;

static int parse_evno(const char* path){
  int n=-1; const char* p=strrchr(path,'t'); // ...event12
  if (p) n=atoi(p+1);
  return n;
}
static int open_grab_path(const char *path){
  int fd = open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) return -1;
  ioctl(fd, EVIOCGRAB, 1);
  return fd;
}

/* ---------------- Detect 1 chuỗi -> signature duy nhất ---------------- */
typedef struct {
  char action[32];     // label người đọc
  char signature[64];  // KEY duy nhất để gán
} Detect;

static void detect_sequence(SelFD *sfds, int nfds, Detect *out){
  // Tập hợp các tính chất thu được cho từng fd (để biết đến từ E11 hay E12)
  typedef struct {
    int dx, dy;
    int saw_btn, btn_down, btn_up;
    int last_key_code;   // mã EV_KEY cuối (vd KEY_VOLUMEDOWN)
    int saw_msc;         // có EV_MSC/MSC_SCAN?
    unsigned int msc_val;
    long long down_t0;
    long long last_time;
    long long start_time;
  } Acc;
  Acc acc[MAX_SEL]; memset(acc,0,sizeof(acc));

  struct pollfd pf[MAX_SEL];
  for(int i=0;i<nfds;i++){ pf[i].fd=sfds[i].fd; pf[i].events=POLLIN; pf[i].revents=0; }

  long long last_any = 0;
  for(;;){
    int r = poll(pf, nfds, 3000);     // chờ lần bấm đầu tối đa 3s
    if (r<=0) break;
    for(int i=0;i<nfds;i++){
      if (!(pf[i].revents & POLLIN)) continue;
      struct input_event ev; ssize_t n = read(pf[i].fd, &ev, sizeof(ev));
      if (n != sizeof(ev)) continue;
      long long t = now_ms();
      if (acc[i].start_time==0) acc[i].start_time=t;
      acc[i].last_time=t; last_any=t;

      if (ev.type==EV_KEY){
        acc[i].last_key_code = ev.code;
        if (ev.code==BTN_MOUSE){
          acc[i].saw_btn=1;
          if (ev.value==1){ acc[i].btn_down=1; acc[i].down_t0=t; }
          if (ev.value==0){ acc[i].btn_up=1; }
        }
      } else if (ev.type==EV_MSC && ev.code==MSC_SCAN){
        acc[i].saw_msc=1; acc[i].msc_val = (unsigned int)ev.value;
      } else if (ev.type==EV_REL){
        if (ev.code==REL_X){ if (abs(ev.value)>=DEADZONE) acc[i].dx += ev.value; }
        if (ev.code==REL_Y){ if (abs(ev.value)>=DEADZONE) acc[i].dy += ev.value; }
      }

      if (ev.type==EV_SYN && ev.code==SYN_REPORT){
        // khoảng lặng cho từng fd: xử lý ở ngoài theo last_any
      }
    }
    if (last_any && (now_ms() - last_any > GAP_MS)) break;
  }

  // Ưu tiên: KEY/MSC từ E11 (Consumer Control) vì chúng là phím media/volume rõ ràng
  for(int i=0;i<nfds;i++){
    if (strstr(sfds[i].name,"Consumer Control") || strstr(sfds[i].name,"Consumer")){
      if (acc[i].saw_msc){
        snprintf(out->action,sizeof(out->action),"MSC_SCAN");
        snprintf(out->signature,sizeof(out->signature),"%s:MSC:%08x", sfds[i].tag, acc[i].msc_val);
        return;
      }
      if (acc[i].last_key_code){
        // Ví dụ KEY_POWER, KEY_VOLUMEDOWN/UP...
        snprintf(out->action,sizeof(out->action),"KEY_%d", acc[i].last_key_code);
        snprintf(out->signature,sizeof(out->signature),"%s:KEY:%d", sfds[i].tag, acc[i].last_key_code);
        return;
      }
    }
  }

  // Tiếp theo: BTN_MOUSE (camera) ở E12 (Mouse)
  for(int i=0;i<nfds;i++){
    if (acc[i].saw_btn){
      long long dt = (acc[i].btn_down && acc[i].btn_up) ? (acc[i].last_time - acc[i].down_t0) : 0;
      if (dt >= HOLD_MS){
        snprintf(out->action,sizeof(out->action),"CAMERA_HOLD");
        snprintf(out->signature,sizeof(out->signature),"%s:BTN_MOUSE:HOLD", sfds[i].tag);
      } else {
        snprintf(out->action,sizeof(out->action),"CAMERA_TAP");
        snprintf(out->signature,sizeof(out->signature),"%s:BTN_MOUSE:TAP", sfds[i].tag);
      }
      return;
    }
  }

  // Cuối cùng: D-pad bằng REL_X/Y (Mouse)
  for(int i=0;i<nfds;i++){
    int dx = acc[i].dx, dy = acc[i].dy;
    if (dx==0 && dy==0) continue;
    long long dur = acc[i].last_time - acc[i].start_time;
    if (abs(dx) > abs(dy)){
      if (dx > 0){
        snprintf(out->action,sizeof(out->action),"RIGHT_TAP");
        snprintf(out->signature,sizeof(out->signature),"%s:REL:X:+", sfds[i].tag);
      } else {
        snprintf(out->action,sizeof(out->action),"LEFT_TAP");
        snprintf(out->signature,sizeof(out->signature),"%s:REL:X:-", sfds[i].tag);
      }
      return;
    }else{
      if (dy < 0){
        if (dur>=HOLD_MS){
          snprintf(out->action,sizeof(out->action),"UP_HOLD");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:-:HOLD", sfds[i].tag);
        }else{
          snprintf(out->action,sizeof(out->action),"UP_TAP");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:-:TAP", sfds[i].tag);
        }
      }else{
        if (dur>=HOLD_MS){
          snprintf(out->action,sizeof(out->action),"DOWN_HOLD");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:+:HOLD", sfds[i].tag);
        }else{
          snprintf(out->action,sizeof(out->action),"DOWN_TAP");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:+:TAP", sfds[i].tag);
        }
      }
      return;
    }
  }

  // Không nhận dạng được
  snprintf(out->action,sizeof(out->action),"UNKNOWN");
  snprintf(out->signature,sizeof(out->signature),"UNKNOWN");
}

/* ---------------- Bảng gán signature -> keycode ---------------- */
typedef struct {
  char sig[64];
  int  keycode;  // 131..142 = F1..F12
  char name[16];
} MapItem;
#define MAX_MAP 64
static MapItem maps[MAX_MAP];
static int nmap=0;

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
  for (int i=0;i<nmap;i++) if (!strcmp(maps[i].sig, sig)) return i;
  return -1;
}

/* ---------------- RUN loop ---------------- */
static void run_loop(SelFD *sfds, int nfds){
  printf("\n[RUN] Nhấn phím — tôi in & phát key đã gán. (Ctrl+C để thoát)\n");
  long long last_emit=0;
  while(1){
    Detect d={0};
    detect_sequence(sfds,nfds,&d);
    if (!d.signature[0]) continue;
    int idx=find_map(d.signature);
    if (idx<0){ printf("→ %s (sig=%s) CHƯA GÁN\n", d.action, d.signature); continue; }
    if (now_ms()-last_emit < COOLDOWN_MS) continue;
    last_emit=now_ms();
    printf("→ %s  =>  %s (%d)\n", d.action, maps[idx].name, maps[idx].keycode);
    char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", maps[idx].keycode);
    shf("%s", cmd);
  }
}

/* ---------------- main ---------------- */
int main(void){
  setvbuf(stdout,NULL,_IONBF,0);
  setvbuf(stderr,NULL,_IONBF,0);

  // Liệt kê (scan trực tiếp, nếu rỗng thì fallback getevent -S)
  scan_by_dir();
  if (ndev==0) scan_by_getevent();
  if (ndev==0){ fprintf(stderr,"Không liệt kê được thiết bị input.\n"); return 1; }

  printf("== Input devices ==\n");
  for (int i=0;i<ndev;i++) printf("[%d] %-28s %s\n", i, devs[i].name, devs[i].path);
  printf("\nChọn thiết bị bằng chỉ số (cách nhau bởi dấu cách), rồi Enter.\n");
  printf("Gợi ý: chọn cả \"BLE-M3 Mouse\" và \"BLE-M3 Consumer Control\".\n> ");

  // Đọc lựa chọn
  int sel_idx[MAX_SEL], nsel=0; char line[256];
  if (!fgets(line,sizeof(line),stdin)) return 1;
  for (char *p=strtok(line," \t\r\n"); p && nsel<MAX_SEL; p=strtok(NULL," \t\r\n"))
    sel_idx[nsel++]=atoi(p);
  if (nsel==0){ printf("Không chọn thiết bị nào.\n"); return 1; }

  // Mở & GRAB
  SelFD sfds[MAX_SEL]; int nfds=0;
  for (int i=0;i<nsel;i++){
    int k=sel_idx[i]; if (k<0||k>=ndev) continue;
    int fd=open_grab_path(devs[k].path);
    if (fd<0){ printf("Không mở được: %s\n", devs[k].path); continue; }
    sfds[nfds].fd=fd;
    snprintf(sfds[nfds].name,sizeof(sfds[nfds].name),"%s", devs[k].name);
    snprintf(sfds[nfds].path,sizeof(sfds[nfds].path),"%s", devs[k].path);
    sfds[nfds].evno = parse_evno(devs[k].path);
    snprintf(sfds[nfds].tag,sizeof(sfds[nfds].tag),"E%d", sfds[nfds].evno);
    printf("Grabbed: %s (%s)  tag=%s\n", sfds[nfds].path, sfds[nfds].name, sfds[nfds].tag);
    nfds++;
  }
  if (nfds==0){ printf("Không mở được thiết bị nào.\n"); return 1; }

  // Học / gán
  for(;;){
    Detect d1={0}, d2={0};
    printf("\n[LEARN] Bấm 1 phím trên remote...\n");
    detect_sequence(sfds,nfds,&d1);
    if (!d1.signature[0]){ printf("Không nhận được chuỗi.\n"); continue; }
    printf("→ Thấy: %s (sig=%s)\n", d1.action, d1.signature);

    printf("Bấm lại phím đó để xác nhận...\n");
    detect_sequence(sfds,nfds,&d2);
    if (strcmp(d1.signature,d2.signature)!=0){
      printf("Không khớp! Lần 2: %s (sig=%s)\n", d2.action, d2.signature);
      continue;
    }
    printf("OK, đã khớp: %s\n", d1.action);

    // Nhập keycode
    printf("Nhập mã phím để gán (131..142 = F1..F12), hoặc 's' để bỏ qua: ");
    if (!fgets(line,sizeof(line),stdin)) break;
    if (line[0]=='s'||line[0]=='S') continue;
    int code=atoi(line); if (code<=0){ printf("Mã không hợp lệ.\n"); continue; }

    int idx=find_map(d1.signature);
    if (idx<0 && nmap<MAX_MAP){ idx=nmap++; snprintf(maps[idx].sig,sizeof(maps[idx].sig),"%s", d1.signature); }
    if (idx<0){ printf("Hết slot map.\n"); continue; }
    maps[idx].keycode=code; snprintf(maps[idx].name,sizeof(maps[idx].name),"%s", keyname(code));
    printf("→ GÁN: %s  =>  %s (%d)\n", d1.action, maps[idx].name, maps[idx].keycode);

    // Test nhanh
    printf("Bấm lại để kiểm tra (sẽ phát key nếu khớp)...\n");
    Detect dt={0}; detect_sequence(sfds,nfds,&dt);
    if (!strcmp(dt.signature,d1.signature)){
      printf("Khớp! Phát: %s (%d)\n", maps[idx].name, maps[idx].keycode);
      char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", maps[idx].keycode);
      shf("%s", cmd);
    } else {
      printf("Không khớp khi test (thấy %s / %s)\n", dt.action, dt.signature);
    }

    // Vào RUN hay học tiếp
    printf("\nNhập 'r' để RUN, 'q' để thoát, phím khác để học tiếp: ");
    if (!fgets(line,sizeof(line),stdin)) break;
    if (line[0]=='r'||line[0]=='R') run_loop(sfds,nfds);
    else if (line[0]=='q'||line[0]=='Q') break;
  }

  for (int i=0;i<nfds;i++){ ioctl(sfds[i].fd,EVIOCGRAB,0); close(sfds[i].fd); }
  return 0;
}
