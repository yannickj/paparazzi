/*
 * Copyright (C) VERDU Titouan
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/lwc_sensor/lwc_sensor.c"
 * @author VERDU Titouan
 * This module will do some action on the flight plan according to the value of LWC given by the python agent based on the MesoNh cloud simulation.
 */

#include "modules/lwc_sensor/lwc_sensor.h"

#include "std.h"
#include "paparazzi.h"
#include "state.h"
#include "subsystems/navigation/common_nav.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include "modules/mission/mission_common.h"

uint8_t test;
float lwc_value;

int IvytoInt(uint8_t * buffer) {
	int i, res;
	res = 0;
	 
	for (i = 0; i <  DL_PAYLOAD_COMMAND_command_length(buffer); i++) {
	  res = res * 10 +  DL_PAYLOAD_COMMAND_command(dl_buffer)[i];
	}
	 
	return res;
}

float denormalized(int value){
	float res1 = (float) value/255.0;
	float res = (float) res1*0.6;
	return res;
}

void lwc_sensor_init(void) {
	test = 0;
	lwc_value = 0.f;
}

void LWC_sensor_in_out_callback(void) {

	if(DL_PAYLOAD_COMMAND_ac_id(dl_buffer) == AC_ID){

		test = IvytoInt(dl_buffer);
		lwc_value = denormalized(test);
		struct EnuCoor_f *pos = stateGetPositionEnu_f();
		//struct FloatEulers *att = stateGetNedToBodyEulers_f();

		//printf("Phi: %f ; Psi: %f ; Theta: %f \n", &(att->phi), &(att->psi), &(att->theta));

		printf("Dir : %f \n", stateGetHorizontalSpeedDir_f());

		if (lwc_value > 0.f){
			printf("X: %f ; Y: %f ; Z: %f ; lwc : %.6f \n", &(pos->x), &(pos->y), &(pos->z), lwc_value);
			//printf("Phi: %f ; Psi: %f ; Theta: %f \n", &(att->phi), &(att->psi), &(att->theta));	
		}
	}	
}

struct _mission_element me;
  me.type = MissionCircle;
  me.element.mission_circle.center.center_f.x = DL_MISSION_CIRCLE_center_east(dl_buffer);
  me.element.mission_circle.center.center_f.y = DL_MISSION_CIRCLE_center_north(dl_buffer);
  me.element.mission_circle.center.center_f.z = DL_MISSION_CIRCLE_center_alt(dl_buffer);
  me.element.mission_circle.radius = DL_MISSION_CIRCLE_radius(dl_buffer);
  me.duration = DL_MISSION_CIRCLE_duration(dl_buffer);
  me.index = DL_MISSION_CIRCLE_index(dl_buffer);

  enum MissionInsertMode insert = (enum MissionInsertMode)(DL_MISSION_CIRCLE_insert(dl_buffer));

  return mission_insert(insert, &me);

