/*
 * Copyright (C) 2017  Xavier Paris
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file gpsd2ivy_resetWPT.c
 *  \brief Reset state from gpsd
 *
 *   This receives position information through ivy bus and request on board reset state ref
 *   (moving wpt dummy = 0)
 */

#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <Ivy/ivy.h>

#define UPDATE_PERIOD 10000.0

int ac_id=3;
int wpt_nb=0;

struct timeval t1, t2;

static void on_Position(IvyClientPtr app, void *user_data, int argc, char *argv[])
{
  double elapsedTime;

  gettimeofday(&t2, NULL);

  elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
  elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

  printf("[%f]\n",elapsedTime);

  if(elapsedTime >= UPDATE_PERIOD) {
    // latitude, longitude, altitude
    IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f",
        ac_id,wpt_nb,atof(argv[3]),atof(argv[4]),atof(argv[7]));
      
    printf("send \n");
    memcpy(&t1,&t2,sizeof(struct timeval));
  }
}

int main(int argc, char** argv)
{
  char* ivy_bus;
  int ret=0;

  if(argc==2) ac_id=atoi(argv[1]);
  else ret=-1; 

  if(ret==-1) {
    printf("Usage:\ngpsd2ivy_resetWPT ac_id\n");
    exit(0);
  }

  memset(&t1, 0, sizeof(struct timeval));

#ifdef __APPLE__
  ivy_bus = "224.255.255.255";
#else
  ivy_bus = "127.255.255.255";
#endif

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("gpsd2ivy_resetWPT", "gpsd2ivy_resetWPT READY", NULL, NULL, NULL, NULL);

  IvyBindMsg(on_Position, NULL, "^ground FLIGHT_PARAM GCS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (.*)");

  IvyStart(ivy_bus);

  g_main_loop_run(ml);

  return 0;
}
