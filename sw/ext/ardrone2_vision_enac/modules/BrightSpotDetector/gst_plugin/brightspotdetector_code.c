
#include <unistd.h>     // usleep
#include <string.h>     // memset
#include <gst/gst.h>    // gprint

#include "brightspotdetector_code.h"
#include "cv_brightspot.h"

#include "paparazzi.h"

////////////////////////////////////////////////
// Plugin inputs
unsigned int imgWidth, imgHeight;
unsigned int tcp_port;
unsigned char threshtune;


void my_plugin_init()
{
  paparazzi_message_server_start();
}


////////////////////////////////////////////////
// PLUGIN CHAIN FUNCTION

void my_plugin_run(unsigned char* frame)
{
  signed int blobP[8];
  memset(blobP, 0, sizeof(blobP[0]) * 8);
  unsigned int max_idx, max_idy;

  // Run Actual Payload
  brightspotDetector(frame,blobP,&max_idx,&max_idy);

  // Store the result
  gst2ppz.blob_x1 = blobP[0];
  gst2ppz.blob_y1 = blobP[1];
  gst2ppz.blob_x2 = blobP[2];
  gst2ppz.blob_y2 = blobP[3];
  gst2ppz.blob_x3 = blobP[4];
  gst2ppz.blob_y3 = blobP[5];
  gst2ppz.blob_x4 = blobP[6];
  gst2ppz.blob_y4 = blobP[7];
  gst2ppz.counter++;

  // Send
  paparazzi_message_send();
}





