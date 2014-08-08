

#include <stdlib.h>     // calloc, exit, free
#include <unistd.h>     // usleep
#include <stdio.h>      // printf
#include <pthread.h>    // pthread_create
#include <string.h>     // memset


#include "ardrone_pprz.h"

//////////////////////////////////////////////
// GST PLUGIN SETTINGS

unsigned int imgWidth, imgHeight;
int mode;
gint adjust_factor;
unsigned int tcpport;

//////////////////////////////////////////////
// PPRZ COMMUNICATIONS

#include "video_message_structs_sky.h"
#include "socket.h"

unsigned int socketIsReady;
struct gst2ppz_message_struct_sky gst2ppz;
struct ppz2gst_message_struct_sky ppz2gst;







//////////////////////////////////////////////
// INIT


#include "optic_flow.h"
#include "trig.h"

//optical flow
unsigned char * old_img;
int old_pitch,old_roll,old_alt;


unsigned int counter;

float opt_angle_y_prev;
float opt_angle_x_prev;
void makeCross(unsigned char * img, int x,int y, int imw, int imh);
void *TCP_threat( void *ptr);


void my_plugin_init(void)
{
  counter = 0;
  old_img = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
  old_pitch = 0;
  old_roll = 0;
  old_alt=0;
  ppz2gst.pitch = 0;
  ppz2gst.roll = 0;
  opt_angle_y_prev = 0;
  opt_angle_x_prev=0;


  if (tcpport>0)
  {
    //start seperate threat to connect
    //seperate threat is needed because otherwise big delays can exist in the init or chain function
    //causing the gst to crash

    pthread_t th1;
    int th1_r;
    pthread_create(&th1,NULL,TCP_threat,&th1_r);
  }
}


void *TCP_threat( void *ptr)
{
  g_print("Waiting for connection on port %d\n",tcpport);
  socketIsReady = initSocket(tcpport);
  if (!socketIsReady) {
    g_print("Error initialising connection\n");
  } else {
    g_print("Connected!\n");
  }


  while(1) {
    int res = Read_msg_socket((char *) &ppz2gst,sizeof(ppz2gst));
    if	(res>1) {
      int tmp;
      tmp = (int)counter - (int)ppz2gst.counter;
      if (tmp>6) {
        g_print("Current counter: %d, Received counter: %d, diff: %d\n",counter, ppz2gst.counter, tmp); //delay of 3 is caused by the fact not every frame is used (15fps mod 3)
      }

    } else {
      g_print("Nothing received: %d\n",res);
      usleep(100000);
    }
  }
}



void my_plugin_run(unsigned char* img)
{

  //if GST_BUFFER_SIZE(buf) <> imgheight*imgwidth*2 -> wrong color space!!!
  if (mode==2)
  {
    int MAX_POINTS, error;
    int n_found_points,mark_points;
    int *x, *y, *new_x, *new_y, *status;
    mark_points = 0;

    //save most recent values of attitude for the currently available frame
    int current_pitch = ppz2gst.pitch;
    int current_roll = ppz2gst.roll;
    int current_alt = ppz2gst.alt;

    x = (int *) calloc(40,sizeof(int));
    new_x = (int *) calloc(40,sizeof(int));
    y = (int *) calloc(40,sizeof(int));
    new_y = (int *) calloc(40,sizeof(int));
    status = (int *) calloc(40,sizeof(int));


    if (tcpport==0) {
      //test code if no network ppz communication is available
      current_alt = 100;
      current_pitch=0;
      current_roll=0;
    }


    MAX_POINTS = 40;


    //active corner:
    int *active;
    active =(int *) calloc(40,sizeof(int));
    int GRID_ROWS = 5;
    int ONLY_STOPPED = 0;
    error = findActiveCorners(img, GRID_ROWS, ONLY_STOPPED, x, y, active, &n_found_points, mark_points,imgWidth,imgHeight);

    /*
		//normal corner:
		int suppression_distance_squared;
		suppression_distance_squared = 3 * 3;
		error = findCorners(img, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points,imgWidth,imgHeight);
     */

    if(error == 0)
    {
      error = opticFlowLK(img, old_img, x, y, n_found_points, imgWidth, imgHeight, new_x, new_y, status, 5, MAX_POINTS);


      //calculate roll and pitch diff:
      float diff_roll = (float)(current_roll- old_roll)/36.0; // 72 factor is to convert to degrees
      float diff_pitch = (float)(current_pitch- old_pitch)/36.0;

      //calculate mean altitude between the to samples:
      int mean_alt;
      if (current_alt>old_alt)
        mean_alt = (current_alt-old_alt)/2 + old_alt;
      else
        mean_alt = (old_alt-current_alt)/2 + current_alt;


      //remember the frame and meta info
      memcpy(old_img,img,imgHeight*imgWidth*2);
      old_pitch = current_pitch;
      old_roll = current_roll;
      old_alt = current_alt;



      if(error == 0)
      {
        showFlow(img, x, y, status, n_found_points, new_x, new_y, imgWidth, imgHeight);

        int tot_x=0;
        int tot_y=0;
        for (int i=0; i<n_found_points;i++) {
          tot_x = tot_x+(new_x[i]-x[i]);
          tot_y = tot_y+(new_y[i]-y[i]);
        }

        //convert pixels/frame to degrees/frame
        float scalef = 64.0/400.0; //64 is vertical camera diagonal view angle (sqrt(320²+240²)=400)
        float opt_angle_x = tot_x*scalef; //= (tot_x/imgWidth) * (scalef*imgWidth); //->degrees/frame
        float opt_angle_y = tot_y*scalef;

        if (abs(opt_angle_x-opt_angle_x_prev)> 3.0) {
          opt_angle_x = opt_angle_x_prev;
        } else	{
          opt_angle_x_prev = opt_angle_x;
        }

        if (abs(opt_angle_y-opt_angle_y_prev)> 3.0) {
          opt_angle_y = opt_angle_y_prev;
        } else	{
          opt_angle_y_prev = opt_angle_y;
        }


        //g_print("Opt_angle x: %f, diff_roll: %d; result: %f. Opt_angle_y: %f, diff_pitch: %d; result: %f. Height: %d\n",opt_angle_x,diff_roll,opt_angle_x-diff_roll,opt_angle_y,diff_pitch,opt_angle_y-diff_pitch,mean_alt);


        //compensate optic flow for attitude (roll,pitch) change:
        opt_angle_x -=  diff_roll;
        opt_angle_y -= diff_pitch;

        //calculate translation in cm/frame from optical flow in degrees/frame
        float opt_trans_x = (float)tan_zelf(opt_angle_x)/1000.0*(float)mean_alt;
        float opt_trans_y = (float)tan_zelf(opt_angle_y)/1000.0*(float)mean_alt;

        g_print("%f;%f;%f;%f;%f;%f;%d;%f;%f\n",opt_angle_x+diff_roll,diff_roll,opt_angle_x,opt_angle_y+diff_pitch,diff_pitch,opt_angle_y,mean_alt,opt_trans_x,opt_trans_y);

        if (tcpport>0) { 	//if network was enabled by user
          if (socketIsReady) {
            gst2ppz.counter = counter;
            gst2ppz.optic_flow_x = tot_x;
            gst2ppz.optic_flow_y = tot_y;
            Write_msg_socket((char *) &gst2ppz, sizeof(gst2ppz));
          }
        }


      } else g_print("error1\n");
    } else g_print("error2\n");

    free(x);
    free(new_x);
    free(y);
    free(new_y);
    free(status);
    free(active);



    //		if (filter->silent == FALSE) {
    //			g_print("Errorh: %d, n_found_points: %d\n",error,n_found_points);
    //		}

  }

  counter++; // to keep track of data through ppz communication

}



