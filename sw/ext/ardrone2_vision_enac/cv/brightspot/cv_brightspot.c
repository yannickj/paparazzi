

#include <stdlib.h>     // calloc
#include <string.h>     // memset

#include "cv_brightspot.h"
#include "brightspotdetector_code.h"

const int dx[] = {+1, 0, -1, 0};
const int dy[] = {0, +1, 0, -1};
int label[320][240];
unsigned int area = 0;
unsigned int sumX = 0;
unsigned int sumY = 0;
unsigned int minArea = 400;

#define image_index(xx, yy)  ((yy * imgWidth + xx) * 2)



void brightspotDetector(unsigned char *frame_buf, int blob[], unsigned int * max_idx,unsigned int * max_idy)
{
  unsigned char thresh;
  unsigned int * OneDHist =(unsigned int *) calloc(256,sizeof(unsigned int));
  unsigned int * hist_x =(unsigned int *) calloc(imgWidth,sizeof(unsigned int));
  unsigned int * hist_y = (unsigned int *)calloc(imgHeight,sizeof(unsigned int));
  get1DHist(frame_buf,OneDHist);
  thresh = getThreshold(OneDHist);
  createBinaryImage(thresh,frame_buf);
  //get2DHist(frame_buf,hist_x,hist_y);
  //*max_idx = getMedian(hist_x,imgWidth);
  //*max_idy = getMedian(hist_y,imgHeight);
  unsigned int i, j, ix;
  unsigned int component = 0;
  area = 0;
  sumX = 0;
  sumY = 0;
  unsigned int blobAccepted = 0;
  unsigned int blobOutlier = 0;
  unsigned int totalLabel = 0;

  for (i = 0; i < imgWidth; ++i)
  {
    for (j = 0; j < imgHeight; ++j)
    {
      ix = image_index(i,j);
      if (!label[i][j] && frame_buf[ix+1])
      {
        totalLabel++;
        blobLabeling(frame_buf,i, j, ++component);
        if(area<minArea)
        {
          area = 0;
          sumX = 0;
          sumY = 0;
          blobOutlier++;
          continue;
        }
        if(blobAccepted<8){
          //blob in body axis
          blob[blobAccepted] = -((sumY/area)-120);
          blob[blobAccepted+1] = (sumX/area)-160;
        }
        blobAccepted = blobAccepted + 2;
        //g_print("blob %d : area = %d xdel = %d ydel = %d \n", blobAccepted/2, area, -((sumY/area)-120), sumX/area-160);
        area = 0;
        sumX = 0;
        sumY = 0;
      }
    }
  }

  memset(label, 0, sizeof(label[0][0]) * imgWidth * imgHeight);

  free(OneDHist);
  free(hist_x);
  free(hist_y);
}
void get1DHist(unsigned char *frame_buf, unsigned int * OneDHist) {
  unsigned int ix;
  unsigned int color_channels = 4;
  unsigned int step = 1 * color_channels;


  for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
  {
    OneDHist[frame_buf[ix+1]]++;
    OneDHist[frame_buf[ix+3]]++;
  }

}

unsigned char getThreshold(unsigned int * OneDHist) {

  unsigned int total= (unsigned int)((float)(imgWidth*imgHeight)*((float)threshtune/100));
  unsigned char i;
  unsigned int tmptotal = 0;

  for (i = 0; i<255; i++) {
    tmptotal+=OneDHist[i];
    if (tmptotal> total)
      return i;
  }
  return 255;
}

void createBinaryImage(unsigned char threshold, unsigned char * frame_buf) {
  unsigned int ix;
  unsigned int color_channels = 4;
  unsigned int step = 1 * color_channels;

  for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
  {
    //frame_buf[ix] = 0;
    //frame_buf[ix+2] = 0;
    if (frame_buf[ix+1] < threshold)
      frame_buf[ix+1] = 0;
    else
      frame_buf[ix+1] = 255;
    if (frame_buf[ix+3] < threshold)
      frame_buf[ix+3] = 0;
    else
      frame_buf[ix+3] = 255;
  }

}

void get2DHist(unsigned char * frame_buf, unsigned int * hist_x, unsigned int * hist_y) {
  unsigned int x,y,ix;

  for (x=0; x<(imgWidth); x++)
  {
    unsigned int tmpsum = 0;
    for (y=0; y<(imgHeight); y++)
    {
      ix = image_index(x,y);
      tmpsum+=frame_buf[ix+1] > 0;
    }
    hist_x[x] = tmpsum;

  }

  //TODO: optimize loop below to integrate with loop above...
  for (y=0; y<(imgHeight); y++)
  {
    unsigned int tmpsum = 0;
    for (x=0; x<(imgWidth); x++)
    {
      tmpsum+=frame_buf[image_index(x,y) +1] > 0;
    }
    hist_y[y] = tmpsum;
  }

}
unsigned int cmpfunc (const void * a, const void * b)
{
  return ( *(unsigned int*)a - *(unsigned int*)b );
}
unsigned int getMedian(unsigned int * hist,  unsigned int size) {

  unsigned int total = 0;
  unsigned int tmptotal = 0;

  for (unsigned int i = 0 ; i< size; i++) {
    total+=hist[i];
  }

  for (unsigned int i = 0 ; i< size; i++) {
    tmptotal+=hist[i];
    if (tmptotal>total/2)
      return i;
  }
  return 0;
}

/*
void getxy(unsigned int max_y_ix, unsigned int * max_idx, unsigned int * max_idy) {
	max_y_ix/=2;
 *max_idy = (max_y_ix / imgWidth);
 *max_idx = (max_y_ix) - *max_idy*imgWidth;
}
 */
void blobLabeling(unsigned char *frame_buf, unsigned int x, unsigned int y, unsigned int current_label)
{
  unsigned int ix;

  if (x >= imgWidth) return;
  if (y >= imgHeight) return;
  ix = image_index(x,y);
  if (label[x][y] || !frame_buf[ix+1]) return;
  //if (*area > (imgWidth*imgHeight) || *sumX > 4294967295 || *sumY > 4294967295) return;

  label[x][y] = current_label;
  area = area + 1;
  sumX = sumX + x;
  sumY = sumY + y;

  int direction;
  for (direction = 0; direction < 4; ++direction)
    blobLabeling(frame_buf, x + dx[direction], y + dy[direction], current_label);

}


