// blem3_ptt_lefttap.c — Bắt riêng hành động LEFT_TAP (REL_X:-) để gán PTT cho Zello
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <time.h>

#define PTT_KEYCODE 25      // 25 = VOLUME_DOWN ; có thể đổi sang 79 = HEADSETHOOK
#define REL_THRESH 20       // độ lệch tối thiểu nhận LEFT_TAP
#define COOLDOWN_MS 300     // chống lặp khi bấm nhanh
#define READ_TIMEOUT_MS 5000

static long long now_ms(void){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

int main(void){
  setvbuf(stdout,NULL,_IONBF,0);

  printf("== Input devices ==\n");
  for(int i=0;i<64;i++){
    char p[64]; snprintf(p,sizeof(p),"/dev/input/event%d",i);
    int fd=open(p,O_RDONLY|O_CLOEXEC); if(fd<0) continue;
    char n[256];
    if(ioctl(fd,EVIOCGNAME(sizeof(n)),n)>=0)
      printf("[%d] %s\t%s\n",i,n,p);
    close(fd);
  }

  printf("\nChọn thiết bị BLE-M3 Mouse (vd: 12): ");
  char line[32]; if(!fgets(line,sizeof(line),stdin)) return 1;
  int idx=atoi(line);
  char path[64]; snprintf(path,sizeof(path),"/dev/input/event%d",idx);

  int fd=open(path,O_RDONLY|O_CLOEXEC);
  if(fd<0){ perror("open"); return 1; }
  printf("Đang nghe %s — chỉ bắt LEFT_TAP (REL_X:-) để phát PTT (%d)\n", path, PTT_KEYCODE);

  long long last_ptt=0;
  while(1){
    struct pollfd pfd={fd,POLLIN,0};
    if(poll(&pfd,1,READ_TIMEOUT_MS)<=0) continue;
    struct input_event ev;
    if(read(fd,&ev,sizeof(ev))!=sizeof(ev)) continue;

    // Chỉ quan tâm chuỗi: REL_X âm nhỏ + SYN_REPORT sau cùng
    static int relx=0, rely=0;
    if(ev.type==EV_REL){
      if(ev.code==REL_X) relx+=ev.value;
      if(ev.code==REL_Y) rely+=ev.value;
    }
    else if(ev.type==EV_SYN && ev.code==SYN_REPORT){
      if(relx < -REL_THRESH && (now_ms()-last_ptt > COOLDOWN_MS)){
        last_ptt=now_ms();
        printf("[PTT] LEFT_TAP phát keyevent %d\n",PTT_KEYCODE);
        char cmd[64]; snprintf(cmd,sizeof(cmd),"input keyevent %d",PTT_KEYCODE);
        system(cmd);
      }
      relx=rely=0;
    }
  }
  close(fd);
  return 0;
}
