#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include "encoding/rtp.h"
#include "udp/socket.h"


int main(int argc,char ** argv)
{
  printf("Starting video test program!\n");

  // Network Transmit
  struct UdpSocket* sock;
  //#define FMS_UNICAST 0
  //#define FMS_BROADCAST 1
//  sock = udp_socket("192.168.1.71", 5000, 5001, FMS_UNICAST);
  sock = udp_socket("192.168.209.34", 5000, 5001, FMS_UNICAST);
//  sock = udp_socket("192.168.127.129", 5000, 5001, FMS_UNICAST);

  long cnt = 0;
  while (1) {

    // Aquire image
    printf("Aquiring image %ld...\n",cnt);
    cnt++;

    test_rtp_frame(sock);
    unsigned char buff[128];
    int len = udp_read(sock,buff,128);
    if (len > 0) printf("read %d\n",len);
    usleep(20000);
  }

  return 0;
}
