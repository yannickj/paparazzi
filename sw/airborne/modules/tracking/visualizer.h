#ifndef VISUALIZER_KALMAN_H
#define VISUALIZER_KALMAN_H

extern void visualizer_init();

extern void visualizer_write(float tag_tracking_roll, float tag_tracking_pitch, float tag_tracking_climb, float posx, float posy, float posz, float pltpredx, float pltpredy, float pltpredz, float pltposx, float pltposy, float pltposz);


#endif
