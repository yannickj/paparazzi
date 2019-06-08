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
#include "math/pprz_geodetic_float.h"
#include "subsystems/gps.h"

uint8_t test;
float lwc_value;
bool io_cloud;
float curve_radius;
float radius_error = 30;
float rot_angle;
float radius_sign;
int16_t border_point_nr;

struct EnuCoor_f *pos;
struct EnuCoor_f calculated_point;
struct _mission_element me;
float direction;

int running_pattern = 1;

struct EnuCoor_f barycenter;
int nb_border_point = 0;

float dis_min = 100;
float distance;
struct EnuCoor_f *pos_actual;
struct EnuCoor_f target_point;

static int IvytoInt(uint8_t * buffer) {
	int i, res;
	res = 0;
	 
	for (i = 0; i <  DL_PAYLOAD_COMMAND_command_length(buffer); i++) {
	  res = res * 10 +  DL_PAYLOAD_COMMAND_command(dl_buffer)[i];
	}
	 
	return res;
}

static float denormalized(int value){
	float res1 = (float) value/255.0;
	float res = (float) res1*0.6;
	return res;
}

static float change_rep(float dir){
	return M_PI_2-dir;
}

static inline void border_send_shot_position(void){
  // angles in decideg
  int16_t phi = DegOfRad(stateGetNedToBodyEulers_f()->phi * 10.0f);
  int16_t theta = DegOfRad(stateGetNedToBodyEulers_f()->theta * 10.0f);
  int16_t psi = DegOfRad(stateGetNedToBodyEulers_f()->psi * 10.0f);
  // course in decideg
  int16_t course = DegOfRad(stateGetHorizontalSpeedDir_f()) * 10;
  // ground speed in cm/s
  uint16_t speed = stateGetHorizontalSpeedNorm_f() * 10;

  DOWNLINK_SEND_DC_SHOT(DefaultChannel, DefaultDevice,
                        &border_point_nr,
                        &stateGetPositionLla_i()->lat,
                        &stateGetPositionLla_i()->lon,
                        &stateGetPositionLla_i()->alt,
                        &gps.hmsl,
                        &phi,
                        &theta,
                        &psi,
                        &course,
                        &speed,
                        &gps.tow);

  border_point_nr = border_point_nr + 1;
}


static float process_curve_radius(){
	//return (powf(NOMINAL_AIRSPEED,2) / (NAV_GRAVITY * tan(AUTO1_MAX_ROLL))) + radius_error;
	//return (powf(NOMINAL_AIRSPEED,2) / (NAV_GRAVITY * tan(H_CTL_ROLL_MAX_SETPOINT))) + radius_error;
	return DEFAULT_CIRCLE_RADIUS + radius_error;
}

static float calc_dist(struct EnuCoor_f *pos_actual){
	float res = sqrtf(powf((target_point.x - pos_actual->x),2) + powf((target_point.y - pos_actual->y),2));

	return res;
}

static void update_barycenter(struct EnuCoor_f *new_coord){

	if(nb_border_point > 2){
		barycenter.x = ((barycenter.x * (nb_border_point - 1)) + new_coord->x ) / nb_border_point;
		barycenter.y = ((barycenter.y * (nb_border_point - 1)) + new_coord->y ) / nb_border_point;
		barycenter.z = 200 + ground_alt;
	}

	target_point.x = barycenter.x;
	target_point.y = barycenter.y;
	target_point.z = barycenter.z;

	printf("Barycenter :%f %f %f \n", barycenter.x , barycenter.y, barycenter.z );
}

static struct EnuCoor_f update_point(struct EnuCoor_f *pos_actual){
	struct EnuCoor_f new_point;
	target_point.x = truncf(target_point.x + (2 * (target_point.x - pos_actual->x)));
	target_point.y = truncf(target_point.y + (2 * (target_point.y - pos_actual->y)));
	distance = calc_dist(pos_actual);

	while(distance < dis_min){
		target_point.x = truncf(target_point.x + ((1/2) * (target_point.x - pos_actual->x)));
		target_point.y = truncf(target_point.y + ((1/2) * (target_point.y - pos_actual->y)));
		distance = calc_dist(pos_actual);
	}
	new_point.x = target_point.x;
	new_point.y = target_point.y;
	new_point.z = 200 + ground_alt;

	return new_point;
}

static struct EnuCoor_f process_new_point_lace(struct EnuCoor_f *position, float uav_direction){
	struct EnuCoor_f new_point;
	new_point.x = position->x + (cos(rot_angle + uav_direction) * curve_radius);
	new_point.y = position->y + (sin(rot_angle + uav_direction) * curve_radius);
	new_point.z = position->z + ground_alt + 5;

	//ALTITUDE FIXE
	//new_point.z = 200 + ground_alt;

	return new_point;
}

static void insert_new_point_lace(struct EnuCoor_f new_coord, struct _mission_element new_e, float ext_radius){
	new_e.type = MissionCircle;
	new_e.element.mission_circle.center.center_f.x = new_coord.x;
	new_e.element.mission_circle.center.center_f.y = new_coord.y;
	new_e.element.mission_circle.center.center_f.z = new_coord.z;
	new_e.element.mission_circle.radius = radius_sign * ext_radius;
	new_e.duration = 0;
	new_e.index = 1;

	enum MissionInsertMode insert = ReplaceCurrent;
	mission_insert(insert, &new_e);
	printf("Point LC: %.1f ; %.1f ; %.1f inserted \n", new_coord.x, new_coord.y, new_coord.z) ;

	if (radius_sign < 0){
		rot_angle = -M_PI_2;
		radius_sign = 1.0;
	}
	else{
		rot_angle = M_PI_2;
		radius_sign = -1.0;
	}
}

static struct EnuCoor_f process_new_point_rosette(struct EnuCoor_f *position, float uav_direction){
	struct EnuCoor_f new_point;

	if(io_cloud == false){
		target_point.x = barycenter.x;
		target_point.y = barycenter.y;

		new_point = update_point(position);
	}
	else if(io_cloud == true){
		new_point.x = position->x + (cos(rot_angle + uav_direction) * curve_radius);
		new_point.y = position->y + (sin(rot_angle + uav_direction) * curve_radius);
		new_point.z = 200 + ground_alt;
	}
	//new_point.z = position->z + ground_alt ;

	return new_point;
}

static void insert_new_point_rosette_circle(struct EnuCoor_f new_coord, struct _mission_element new_e, float ext_radius){

	new_e.type = MissionCircle;
	new_e.element.mission_circle.center.center_f.x = new_coord.x;
	new_e.element.mission_circle.center.center_f.y = new_coord.y;
	new_e.element.mission_circle.center.center_f.z = new_coord.z;
	new_e.element.mission_circle.radius = radius_sign * ext_radius;
	new_e.duration = 0;
	new_e.index = 1;

	enum MissionInsertMode insert = ReplaceCurrent;
	mission_insert(insert, &new_e);
	printf("Point RC: %.1f ; %.1f ; %.1f inserted \n", new_coord.x, new_coord.y, new_coord.z);
}

static void insert_new_point_rosette_straight(struct EnuCoor_f new_coord, struct _mission_element new_e, struct EnuCoor_f *position){

	// uint8_t ac_id = 24;
	// uint8_t insert = 3;
	// float WPA_x = position->x;
	// float WPA_y = position->y;
	// float WPB_x = new_coord.x;
	// float WPB_y = new_coord.y;
	// float WP_alt = new_coord.z;
	// float duration = 0.0;
	// uint8_t index = 1;

	new_e.type = MissionSegment;
	new_e.element.mission_segment.from.from_f.x = position->x;
	new_e.element.mission_segment.from.from_f.y = position->y;
	new_e.element.mission_segment.from.from_f.z = ground_alt;
	new_e.element.mission_segment.to.to_f.x = new_coord.x;
	new_e.element.mission_segment.to.to_f.y = new_coord.y;
	new_e.element.mission_segment.to.to_f.z = new_coord.z;
  	new_e.duration = 0;
	new_e.index = 1;

	enum MissionInsertMode insert = ReplaceCurrent;

	// DOWNLINK_SEND_MISSION_SEGMENT(DefaultChannel, DefaultDevice,
	// 							&ac_id,
	// 							&insert,
	// 							&WPA_x,
	// 							&WPA_y,
	// 							&WPB_x,
	// 							&WPB_y,
	// 							&WP_alt,
	// 							&duration,
	// 							&index);

	// <message name="MISSION_SEGMENT" id="24" link="forwarded">
 //      <field name="ac_id" type="uint8"/>
 //      <field name="insert" type="uint8" values="APPEND|PREPEND|REPLACE_CURRENT|REPLACE_ALL"/>
 //      <field name="segment_east_1" type="float" unit="m"/>
 //      <field name="segment_north_1" type="float" unit="m"/>
 //      <field name="segment_east_2" type="float" unit="m"/>
 //      <field name="segment_north_2" type="float" unit="m"/>
 //      <field name="segment_alt" type="float" unit="m">altitude above geoid (MSL)</field>
 //      <field name="duration" type="float" unit="s"/>
 //      <field name="index" type="uint8"/>
 //    </message>

	
	mission_insert(insert, &new_e);
	printf("Path RS from (X:%.1f ; Y:%.1f ; Z:%.1f) to (X:%.1f ; Y:%.1f ; Z:%.1f) inserted \n",position->x, position->y, position->z + ground_alt, new_coord.x, new_coord.y, new_coord.z);
}

void lwc_sensor_init(void) {
	test = 0;
	lwc_value = 0.f;
	io_cloud = false; //False out ; True in the cloud 
	curve_radius = process_curve_radius()/2.0;
	radius_sign = -1.0; //First turn to the left!
	rot_angle = M_PI_2;

	border_point_nr = 0;

	//printf("%.1f \n", curve_radius);

	me.type = MissionWP;
  	me.element.mission_wp.wp.wp_f.x = 350.0;
  	me.element.mission_wp.wp.wp_f.y = 300.0;
  	me.element.mission_wp.wp.wp_f.z = 315.0;
  	me.duration = 0;
  	me.index = 1;

  	if(running_pattern == 1){
  		barycenter.x = me.element.mission_wp.wp.wp_f.x;
  		barycenter.y = me.element.mission_wp.wp.wp_f.y;
  		barycenter.z = me.element.mission_wp.wp.wp_f.z;

  		target_point.x = 0.0;
		target_point.y = 0.0;
		target_point.z = 0.0;
  	}

  	enum MissionInsertMode insert_init = Append;

  	bool worked = mission_insert(insert_init, &me);

  	printf("MissionInit : %s \n", worked?"true":"false");
  	printf("AltitudeInit : %.1f \n", ground_alt);
  	printf("Running Pattern : %d \n", running_pattern);
}


void LWC_sensor_in_out_callback(void) {

	if(DL_PAYLOAD_COMMAND_ac_id(dl_buffer) == AC_ID){

		test = IvytoInt(dl_buffer);
		lwc_value = denormalized(test);
		pos = stateGetPositionEnu_f();

		if(running_pattern == 1 && io_cloud == true){

			distance = calc_dist(pos);
			if (distance < dis_min){
				calculated_point = update_point(pos);
				insert_new_point_rosette_straight(calculated_point, me, pos);
			}
		}
		
		if (lwc_value > 0.05 && io_cloud == false){

			direction = change_rep(stateGetHorizontalSpeedDir_f());

			nb_border_point += 1;
			update_barycenter(pos);

			border_send_shot_position();

			if(running_pattern == 0){
				calculated_point = process_new_point_lace(pos,direction);

				insert_new_point_lace(calculated_point, me, curve_radius);
			}
			else if(running_pattern == 1){
				calculated_point = process_new_point_rosette(pos,direction);

				insert_new_point_rosette_straight(calculated_point, me, pos);
			}
			

			io_cloud = true;
		}
		else if (lwc_value <= 0.05 && io_cloud == true){

			pos = stateGetPositionEnu_f();
			direction = change_rep(stateGetHorizontalSpeedDir_f());

			nb_border_point += 1;
			update_barycenter(pos);

			border_send_shot_position();

			if(running_pattern == 0){
				calculated_point = process_new_point_lace(pos,direction);

				insert_new_point_lace(calculated_point, me, curve_radius);
			}
			else if(running_pattern == 1){
				calculated_point = process_new_point_rosette(pos,direction);

				insert_new_point_rosette_circle(calculated_point, me, curve_radius);
			}
			

			io_cloud = false;
		}
	}	
}