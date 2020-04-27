#include "modules/tracking/visualizer.h"
#include <stdlib.h>
#include <stdio.h>
#include "state.h"
#include "subsystems/navigation/waypoints.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"



// saving in a file
char * output_file_command = "sw/airborne/modules/tracking/kalman_command";

void visualizer_init(){
  FILE *f = fopen(output_file_command, "w");
  char * buf = "roll_command ; pitch_command ; climb_command ; phi_real ; theta_real ; psi_real ; posx ; posy ; dronex ; droney ; predx ; predy ; pltx ; plty\n";
  fputs(buf, f);
  fclose(f);
}


void visualizer_write(float tag_tracking_roll, float tag_tracking_pitch, float tag_tracking_climb, float posx, float posy, float pltpredx, float pltpredy, float pltposx, float pltposy){
  struct FloatEulers * angles = stateGetNedToBodyEulers_f();
  struct EnuCoor_f * posDrone = stateGetPositionEnu_f();


  FILE *f = fopen(output_file_command, "a");
  char roll_s[20];
  char pitch_s[20];
  char climb_s[20];
  char phi_s[20];
  char theta_s[20];
  char psi_s[20];
  char posx_s[20];
  char posy_s[20];
  char drone_posx_s[20];
  char drone_posy_s[20];
  char plt_predx_s[20];
  char plt_predy_s[20];
  char plt_posx_s[20];
  char plt_posy_s[20];

  snprintf(roll_s, 20, "%f", tag_tracking_roll);
  snprintf(pitch_s, 20, "%f", tag_tracking_pitch);
  snprintf(climb_s, 20, "%f", tag_tracking_climb);
  snprintf(phi_s, 20, "%f", 180/3.14 * angles->phi);
  snprintf(theta_s, 20, "%f",180/3.14 * angles->theta);
  snprintf(psi_s, 20, "%f", 180/3.14 * angles->psi);
  snprintf(posx_s, 20, "%f", posx);
  snprintf(posy_s, 20, "%f", posy);
  snprintf(drone_posx_s, 20, "%f", posDrone->x);
  snprintf(drone_posy_s, 20, "%f", posDrone->y);
  snprintf(plt_predx_s, 20, "%f", pltpredx);
  snprintf(plt_predy_s, 20, "%f", pltpredy);
  snprintf(plt_posx_s, 20, "%f", pltposx);
  snprintf(plt_posy_s, 20, "%f", pltposy);

  char buf[200];
  strcpy(buf, roll_s);
  strcat(buf, " ; "); 
  strcat(buf, pitch_s);
  strcat(buf, " ; "); 
  strcat(buf, climb_s);
  strcat(buf, " ; "); 
  strcat(buf, phi_s);
  strcat(buf, " ; "); 
  strcat(buf, theta_s);
  strcat(buf, " ; ");
  strcat(buf, psi_s);
  strcat(buf, " ; ");
  strcat(buf, posx_s);
  strcat(buf, " ; ");
  strcat(buf, posy_s);
  strcat(buf, " ; ");
  strcat(buf, drone_posx_s);
  strcat(buf, " ; ");
  strcat(buf, drone_posy_s);
  strcat(buf, " ; ");
  strcat(buf, plt_predx_s);
  strcat(buf, " ; ");
  strcat(buf, plt_predy_s);
  strcat(buf, " ; ");
  strcat(buf, plt_posx_s);
  strcat(buf, " ; ");
  strcat(buf, plt_posy_s);
  strcat(buf, " \n");
  fputs(buf, f);
  fclose(f);
}

