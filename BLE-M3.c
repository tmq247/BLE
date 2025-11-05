// blem3_learn_adb.c — Học phím BLE-M3 qua adb shell.
// - Liệt kê thiết bị (scan /dev/input + fallback getevent -S)
// - Gom 1 chuỗi hành động -> 1 signature duy nhất
// - Phân biệt CLICK bằng tọa độ tương đối lượng tử hóa (qx,qy)
// - Ưu tiên MSC/KEY từ Consumer Control (event11) cho Volume/Media
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

#define MAX_DEV     64
#define MAX_SEL     8
#define HOLD_MS     350
#define GAP_MS      120
#define DEADZONE    2
#define COOLDOWN_MS 120
#define QPIX        16   // lượng tử hóa dx,dy: q = round(delta / QPIX)

typedef struct { char name[256]; char path[64]; } Dev;
static Dev devs[MAX_DEV]; static int ndev=0;

static long long now_ms(void){ struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000; }
static inline void shf(const char *fmt, ...){
  char cmd[256]; va_list ap; va_start(ap,fmt); vsnprintf(cmd,sizeof(cmd),fmt,ap); va_end(ap); system(cmd);
}

/* ---------- liệt kê ---------- */
static void scan_by_dir(void){
  ndev=0; for(int i=0;i<64;i++){ char p[64],n[256]; snprintf(p,sizeof(p),"/dev/input/event%d",i);
    int fd=open(p,O_RDONLY|O_CLOEXEC); if(fd<0) continue;
    if(ioctl(fd,EVIOCGNAME(sizeof(n)),n)>=0){ snprintf(devs[ndev].name,sizeof(devs[ndev].name),"%s",n);
      snprintf(devs[ndev].path,sizeof(devs[ndev].path),"%s",p); ndev++; }
    close(fd); if(ndev>=MAX_DEV) break; } }
static void trim(char*s){ size_t n=strlen(s); while(n&&(s[n-1]=='\n'||s[n-1]=='\r'||s[n-1]==' '||s[n-1]=='\t')) s[--n]=0; }
static void scan_by_getevent(void){
  ndev=0; FILE*fp=popen("/system/bin/getevent -S 2>/dev/null","r"); if(!fp) return; char line[512],nm[256]="";
  while(fgets(line,sizeof(line),fp)){ trim(line);
    if(strncmp(line,"add device",10)==0){ char *p=strstr(line,"/dev/input/event"); if(!p) continue; char path[128]="";
      strncpy(path,p,sizeof(path)-1); if(fgets(line,sizeof(line),fp)){ trim(line); char *q=strchr(line,'"'); if(q){ q++; char *r=strchr(q,'"'); if(r){*r=0; snprintf(nm,sizeof(nm),"%s",q);} } }
      if(nm[0]){ snprintf(devs[ndev].name,sizeof(devs[ndev].name),"%s",nm); snprintf(devs[ndev].path,sizeof(devs[ndev].path),"%s",path);
        ndev++; nm[0]=0; if(ndev>=MAX_DEV) break; } } }
  pclose(fp); }

/* ---------- open & grab ---------- */
typedef struct { int fd; char name[256]; char path[64]; int evno; char tag[8]; } SelFD;
static int parse_evno(const char*path){ const char*t=strrchr(path,'t'); return t?atoi(t+1):-1; }
static int open_grab_path(const char*path){ int fd=open(path,O_RDONLY|O_CLOEXEC); if(fd>=0) ioctl(fd,EVIOCGRAB,1); return fd; }

/* ---------- detect 1 chuỗi -> signature ---------- */
typedef struct { char action[32]; char signature[64]; } Detect;

static void detect_sequence(SelFD *sfds, int nfds, Detect *out){
  typedef struct {
    int dx,dy;                        // tổng dịch chuyển (từ start tới cuối)
    int dx_before_click, dy_before_click; // tổng dịch chuyển tới BTN_MOUSE DOWN
    int saw_btn, btn_down, btn_up;
    int saw_click; long long click_down_t0;
    int last_key_code; int saw_msc; unsigned int msc_val;
    long long start_t, last_t;
  } Acc;
  Acc acc[MAX_SEL]; memset(acc,0,sizeof(acc));
  struct pollfd pf[MAX_SEL]; for(int i=0;i<nfds;i++){ pf[i].fd=sfds[i].fd; pf[i].events=POLLIN; pf[i].revents=0; }

  int saw_any=0; long long last_any=0;
  for(;;){
    int r=poll(pf,nfds,3000); if(r<=0) break;
    for(int i=0;i<nfds;i++){
      if(!(pf[i].revents&POLLIN)) continue;
      struct input_event ev; ssize_t n=read(pf[i].fd,&ev,sizeof(ev)); if(n!=sizeof(ev)) continue;
      saw_any=1; long long t=now_ms(); if(!acc[i].start_t) acc[i].start_t=t; acc[i].last_t=t; last_any=t;

      if(ev.type==EV_KEY){
        acc[i].last_key_code=ev.code;
        if(ev.code==BTN_MOUSE){
          acc[i].saw_btn=1;
          if(ev.value==1){ acc[i].btn_down=1; acc[i].saw_click=1; acc[i].click_down_t0=t;
            acc[i].dx_before_click=acc[i].dx; acc[i].dy_before_click=acc[i].dy; }
          if(ev.value==0){ acc[i].btn_up=1; }
        }
      } else if(ev.type==EV_MSC && ev.code==MSC_SCAN){
        acc[i].saw_msc=1; acc[i].msc_val=(unsigned int)ev.value;
      } else if(ev.type==EV_REL){
        if(ev.code==REL_X){ if(abs(ev.value)>=DEADZONE) acc[i].dx+=ev.value; }
        if(ev.code==REL_Y){ if(abs(ev.value)>=DEADZONE) acc[i].dy+=ev.value; }
      }
    }
    if(last_any && (now_ms()-last_any>GAP_MS)) break;
  }

  // không có gì -> rỗng
  if(!saw_any){ out->action[0]=0; out->signature[0]=0; return; }

  // 1) Ưu tiên event11: MSC/KEY cho volume/media (nút chụp ảnh luân phiên của bạn)
  for(int i=0;i<nfds;i++){
    if(strstr(sfds[i].name,"Consumer")){
      if(acc[i].saw_msc){ snprintf(out->action,sizeof(out->action),"MSC_SCAN");
        snprintf(out->signature,sizeof(out->signature),"%s:MSC:%08x",sfds[i].tag,acc[i].msc_val); return; }
      if(acc[i].last_key_code){ snprintf(out->action,sizeof(out->action),"KEY_%d",acc[i].last_key_code);
        snprintf(out->signature,sizeof(out->signature),"%s:KEY:%d",sfds[i].tag,acc[i].last_key_code); return; }
    }
  }

  // 2) event12: CLICK theo tọa độ tương đối lượng tử hóa tại thời điểm BTN_MOUSE DOWN
  for(int i=0;i<nfds;i++){
    if(acc[i].saw_click){
      int qx = (acc[i].dx_before_click>=0 ? (acc[i].dx_before_click+QPIX/2) : (acc[i].dx_before_click-QPIX/2)) / QPIX;
      int qy = (acc[i].dy_before_click>=0 ? (acc[i].dy_before_click+QPIX/2) : (acc[i].dy_before_click-QPIX/2)) / QPIX;
      long long hold = (acc[i].btn_down && acc[i].btn_up) ? (acc[i].last_t - acc[i].click_down_t0) : 0;
      if(hold>=HOLD_MS){
        snprintf(out->action,sizeof(out->action),"CLICK_HOLD(%d,%d)",qx,qy);
        snprintf(out->signature,sizeof(out->signature),"%s:CLICK:Q:%d:%d:HOLD",sfds[i].tag,qx,qy);
      }else{
        snprintf(out->action,sizeof(out->action),"CLICK(%d,%d)",qx,qy);
        snprintf(out->signature,sizeof(out->signature),"%s:CLICK:Q:%d:%d:TAP",sfds[i].tag,qx,qy);
      }
      return;
    }
  }

  // 3) Di chuyển D-pad (không click): dùng REL_Y/REL_X + tap/hold như cũ
  for(int i=0;i<nfds;i++){
    int dx=acc[i].dx, dy=acc[i].dy; if(dx==0 && dy==0) continue;
    long long dur = acc[i].last_t - acc[i].start_t;
    if(abs(dx)>abs(dy)){
      if(dx>0){ snprintf(out->action,sizeof(out->action),"RIGHT_TAP");
        snprintf(out->signature,sizeof(out->signature),"%s:REL:X:+",sfds[i].tag); }
      else    { snprintf(out->action,sizeof(out->action),"LEFT_TAP");
        snprintf(out->signature,sizeof(out->signature),"%s:REL:X:-",sfds[i].tag); }
      return;
    }else{
      if(dy<0){
        if(dur>=HOLD_MS){ snprintf(out->action,sizeof(out->action),"UP_HOLD");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:-:HOLD",sfds[i].tag); }
        else             { snprintf(out->action,sizeof(out->action),"UP_TAP");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:-:TAP",sfds[i].tag); }
      }else{
        if(dur>=HOLD_MS){ snprintf(out->action,sizeof(out->action),"DOWN_HOLD");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:+:HOLD",sfds[i].tag); }
        else             { snprintf(out->action,sizeof(out->action),"DOWN_TAP");
          snprintf(out->signature,sizeof(out->signature),"%s:REL:Y:+:TAP",sfds[i].tag); }
      }
      return;
    }
  }

  // không nhận diện được
  out->action[0]=0; out->signature[0]=0;
}

/* ---------- map & run ---------- */
typedef struct { char sig[64]; int keycode; char name[16]; } MapItem;
#define MAX_MAP 64
static MapItem maps[MAX_MAP]; static int nmap=0;
static const char* keyname(int c){ switch(c){
  case 131:return"F1"; case 132:return"F2"; case 133:return"F3"; case 134:return"F4";
  case 135:return"F5"; case 136:return"F6"; case 137:return"F7"; case 138:return"F8";
  case 139:return"F9"; case 140:return"F10"; case 141:return"F11"; case 142:return"F12"; default:return"?";}}
static int find_map(const char*sig){ for(int i=0;i<nmap;i++) if(!strcmp(maps[i].sig,sig)) return i; return -1; }

static void run_loop(SelFD*sfds,int nfds){
  printf("\n[RUN] Nhấn phím — sẽ in & phát key đã gán. (Ctrl+C để thoát)\n");
  long long last_emit=0;
  for(;;){
    Detect d={0}; detect_sequence(sfds,nfds,&d); if(!d.signature[0]) continue;
    int idx=find_map(d.signature);
    if(idx<0){ printf("→ %s (sig=%s) CHƯA GÁN\n", d.action, d.signature); continue; }
    if(now_ms()-last_emit<COOLDOWN_MS) continue; last_emit=now_ms();
    printf("→ %s  =>  %s (%d)\n", d.action, maps[idx].name, maps[idx].keycode);
    char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", maps[idx].keycode); shf("%s",cmd);
  }
}

/* ---------- main ---------- */
int main(void){
  setvbuf(stdout,NULL,_IONBF,0); setvbuf(stderr,NULL,_IONBF,0);

  scan_by_dir(); if(ndev==0) scan_by_getevent();
  if(ndev==0){ fprintf(stderr,"Không liệt kê được thiết bị input.\n"); return 1; }

  printf("== Input devices ==\n");
  for(int i=0;i<ndev;i++) printf("[%d] %-28s %s\n", i, devs[i].name, devs[i].path);
  printf("\nChọn thiết bị (ví dụ: 11 12), Enter.\nGợi ý: chọn cả \"BLE-M3 Mouse\" và \"BLE-M3 Consumer Control\".\n> ");

  int sel[MAX_SEL],nsel=0; char line[256]; if(!fgets(line,sizeof(line),stdin)) return 1;
  for(char*p=strtok(line," \t\r\n"); p&&nsel<MAX_SEL; p=strtok(NULL," \t\r\n")) sel[nsel++]=atoi(p);
  if(nsel==0){ printf("Không chọn thiết bị nào.\n"); return 1; }

  SelFD sfds[MAX_SEL]; int nfds=0;
  for(int i=0;i<nsel;i++){ int k=sel[i]; if(k<0||k>=ndev) continue;
    int fd=open_grab_path(devs[k].path); if(fd<0){ printf("Không mở: %s\n",devs[k].path); continue; }
    sfds[nfds].fd=fd; snprintf(sfds[nfds].name,sizeof(sfds[nfds].name),"%s",devs[k].name);
    snprintf(sfds[nfds].path,sizeof(sfds[nfds].path),"%s",devs[k].path);
    sfds[nfds].evno=parse_evno(devs[k].path); snprintf(sfds[nfds].tag,sizeof(sfds[nfds].tag),"E%d",sfds[nfds].evno);
    printf("Grabbed: %s (%s) tag=%s\n", sfds[nfds].path, sfds[nfds].name, sfds[nfds].tag); nfds++; }
  if(nfds==0){ printf("Không mở được thiết bị nào.\n"); return 1; }

  for(;;){
    Detect d1={0}, d2={0};
    printf("\n[LEARN] Bấm 1 phím trên remote...\n");
    detect_sequence(sfds,nfds,&d1); if(!d1.signature[0]){ printf("Không nhận được chuỗi, thử lại...\n"); continue; }
    printf("→ Thấy: %s (sig=%s)\n", d1.action, d1.signature);

    printf("Bấm lại phím đó để xác nhận...\n");
    detect_sequence(sfds,nfds,&d2);
    if(!d2.signature[0] || strcmp(d1.signature,d2.signature)!=0){ printf("Không khớp / không có tín hiệu lần 2.\n"); continue; }
    printf("OK, đã khớp.\n");

    printf("Nhập mã phím để gán (131..142 = F1..F12), hoặc 's' để bỏ qua: ");
    if(!fgets(line,sizeof(line),stdin)) break; if(line[0]=='s'||line[0]=='S') continue;
    int code=atoi(line); if(code<=0){ printf("Mã không hợp lệ.\n"); continue; }

    int idx=find_map(d1.signature);
    if(idx<0 && nmap<MAX_MAP){ idx=nmap++; snprintf(maps[idx].sig,sizeof(maps[idx].sig),"%s",d1.signature); }
    if(idx<0){ printf("Hết slot map.\n"); continue; }
    maps[idx].keycode=code; snprintf(maps[idx].name,sizeof(maps[idx].name),"%s", keyname(code));
    printf("→ GÁN: %s => %s (%d)\n", d1.signature, maps[idx].name, maps[idx].keycode);

    printf("Bấm lại để test...\n");
    Detect dt={0}; detect_sequence(sfds,nfds,&dt);
    if(dt.signature[0] && !strcmp(dt.signature,d1.signature)){
      printf("Khớp! Phát: %s (%d)\n", maps[idx].name, maps[idx].keycode);
      char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d", maps[idx].keycode); shf("%s",cmd);
    } else { printf("Không khớp khi test (thấy %s)\n", dt.signature[0]?dt.signature:"<rỗng>"); }

    printf("\nNhập 'r' để RUN, 'q' để thoát, phím khác để học tiếp: ");
    if(!fgets(line,sizeof(line),stdin)) break;
    if(line[0]=='r'||line[0]=='R') run_loop(sfds,nfds);
    else if(line[0]=='q'||line[0]=='Q') break;
  }

  for(int i=0;i<nfds;i++){ ioctl(sfds[i].fd,EVIOCGRAB,0); close(sfds[i].fd); }
  return 0;
}
