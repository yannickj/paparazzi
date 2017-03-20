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

/** \file reset_state_ref_from_gpsd2ivy.c
 *  \brief Reset state from gpsd
 *
 *   This receives position information through ivy bus and request on board reset state ref
 *   (moving wpt dummy = 0)
 */

#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <Ivy/ivy.h>

int ac_id=3;
int wpt_nb=0;

static void on_Position(IvyClientPtr app, void *user_data, int argc, char *argv[])
{

  // latitude, longitude, altitude
  IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f",
    ac_id,wpt_nb,atof(argv[3]),atof(argv[4]),atof(argv[7]));
}

int main(int argc, char** argv)
{
  char* ivy_bus;
  int ret=0;

  if(argc>1) {
    ac_id=atoi(argv[1]);
    if(argc==3) {
      wpt_nb=atoi(argv[2]);
      if((wpt_nb!=0)||(wpt_nb!=1)) ret=-1;
    } else ret=-1;
  } else ret=-1;

  if(ret==-1) {
    printf("Usage:\nreset_gpsd2ivy ac_id wpt_nb\n wpt_nb optional 0 or 1\n");
    exit(0);
  }

#ifdef __APPLE__
  ivy_bus = "224.255.255.255";
#else
  ivy_bus = "127.255.255.255";
#endif

  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);

  IvyInit ("ResetStateRef", "ResetStateRef READY", NULL, NULL, NULL, NULL);

  IvyBindMsg(on_Position, NULL, "^ground FLIGHT_PARAM GCS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (.*)");

  IvyStart(ivy_bus);

  g_main_loop_run(ml);

  return 0;
}
