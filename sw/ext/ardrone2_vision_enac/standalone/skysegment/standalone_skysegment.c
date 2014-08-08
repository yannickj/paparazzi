#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include "resize.h"
#include "v4l/video.h"
#include "encoding/jpeg.h"
#include "udp/socket.h"

#include "ObstacleAvoidSkySegmentation/obstacleavoidskysegmentation_code.h"
#include "ObstacleAvoidSkySegmentation/video_message_structs.h"


#define DOWNSIZE_FACTOR   8


int main(int argc,char ** argv)
{
  printf("Starting video test program!\n");
  long len;


  // Video Input
  struct vid_struct vid;
  vid.device = (char*)"/dev/video1";
  vid.w=1280;
  vid.h=720;
  len = 1280*720*2; // width * height * nbytes_perpixel
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    return 0;
  }

  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // Video Resizing
  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);

  // Call Plugin Code
  imgWidth = small.w;
  imgHeight = small.h;
  adjust_factor = 5;
  verbose = 0;
  my_plugin_init();


  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);


  // Network Transmit
  struct UdpSocket* sock;
  //#define FMS_UNICAST 0
  //#define FMS_BROADCAST 1
  sock = udp_socket("192.168.1.2", 5000, 5001, FMS_UNICAST);

  while (1) {

    // Aquire image
    printf("Aquiring an image ...\n");
    video_grab_image(&vid, img_new);


    // Resize: device by 4
    resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);


    // Processing
    ppz2gst.roll += 71;
    if (ppz2gst.roll > 35*70)
      ppz2gst.roll = -35*70;
    my_plugin_run(small.buf);


    // JPEG encode the image:
    uint32_t quality_factor = 1; // quality factor from 1 (high quality) to 8 (low quality)
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf+10, quality_factor, image_format, small.w, small.h, 1);
    uint32_t size = end-(jpegbuf+10);
    create_svs_jpeg_header(jpegbuf,size,small.w);
    printf("Sending an image ...%u\n",size);

    udp_write(sock, jpegbuf, size+10);

  }

  video_close(&vid);

  return 0;
}
