#include "modules/tracking/visualizer.h"
#include <stdlib.h>
#include <stdio.h>
#include "state.h"

// saving in a file
char * output_file_command = "sw/airborne/modules/tracking/kalman_command";

void visualizer_init(){
  FILE *f = fopen(output_file_command, "w");
  char * buf = "roll_command ; pitch_command ; climb_command ; phi_real ; theta_real ; psi_real\n";
  fputs(buf, f);
  fclose(f);
}


void visualizer_write(float tag_tracking_roll, float tag_tracking_pitch, float tag_tracking_climb){
  struct FloatEulers * angles = stateGetNedToBodyEulers_f();
  FILE *f = fopen(output_file_command, "a");
  char roll_s[20];
  char pitch_s[20];
  char climb_s[20];
  char phi_s[20];
  char theta_s[20];
  char psi_s[20];
  snprintf(roll_s, 20, "%f", tag_tracking_roll);
  snprintf(pitch_s, 20, "%f", tag_tracking_pitch);
  snprintf(climb_s, 20, "%f", tag_tracking_climb);
  snprintf(phi_s, 20, "%f", 180/3.14 * angles->phi);
  snprintf(theta_s, 20, "%f",180/3.14 * angles->theta);
  snprintf(psi_s, 20, "%f", 180/3.14 * angles->psi);
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
  strcat(buf, " \n");
  fputs(buf, f);
  fclose(f);
}

