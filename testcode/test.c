#include <unistd.h>
#include <stdio.h>

unsigned char buf[5000];

int main(int argc, char **argv) {
  int cue =0;
  while (1) {
    buf[0]=0xff;
    buf[1]=0xff;
    for (int i=0; i<2400; i++) {
      buf[2 + i*2]= ((cue)&31)==0?0x7c:0x00;
      buf[3 + i*2]= ((cue)&31)==0?0x00:0x00;
    }
    buf[2+2400*2]=0x80;
    cue++;
    write(STDOUT_FILENO, buf,4803);
    usleep(20*1000);
  }
}
