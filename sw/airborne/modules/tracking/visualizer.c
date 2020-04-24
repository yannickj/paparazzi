#include "modules/tracking/visualizer.h"

// saving in a file
char * output_file_command = "kalman_command";

void visualizer_init(){
  FILE *f = fopen(output_file_command, "w");
  fclose(f);
}

void visualizer_write(float tag_tracking_roll, float tag_tracking_pitch, float tag_tracking_climb){
  FILE *f = fopen(output_file_command, "a");

  char roll_s[20];
  char pitch_s[20];
  char climb_s[20];
  snprintf(roll_s, 20, "%f", tag_tracking_roll);
  snprintf(pitch_s, 20, "%f", tag_tracking_pitch);
  snprintf(climb_s, 20, "%f", tag_tracking_climb);
  char buf[200];
  strcpy(buf, roll_s);
  strcat(buf, " ; "); 
  strcat(buf, pitch_s);
  strcat(buf, " ; "); 
  strcat(buf, climb_s);
  strcat(buf, " \n");
  fputs(buf, f);
  fclose(f);
}
