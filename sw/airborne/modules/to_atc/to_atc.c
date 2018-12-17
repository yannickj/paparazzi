#include "generated/airframe.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "subsystems/gps.h"



#ifndef TO_ATC_FLIGHT
#define TO_ATC_FLIGHT "525"
#endif
#ifndef TO_ATC_CALLSIGN
#define TO_ATC_CALLSIGN "DRONE1"
#endif
#ifndef TO_ATC_TACODE
#define TO_ATC_TACODE "0000"
#endif
#ifndef TO_ATC_COMPANY
#define TO_ATC_COMPANY "AFR"
#endif
#ifndef TO_ATC_MODEL
#define TO_ATC_MODEL "A320"
#endif


const char str[5] = TO_ATC_TACODE;
const char *elts[] = { &str[0], &str[1], &str[2], &str[3] , &str[4] };

char tacode[] = TO_ATC_TACODE;


void to_atc_init(void)
{
}


void to_atc_periodic(void)
{
  char  flight[]   = TO_ATC_FLIGHT;
  char  callsign[] = TO_ATC_CALLSIGN;

  float lat        = (stateGetPositionLla_i()->lat)/1e7;
  float lon        = (stateGetPositionLla_i()->lon)/1e6;
  float alt        = stateGetPositionEnu_f()->z + (state.ned_origin_i.hmsl/1e3);

  float track      = DegOfRad(gps.course/1e7);
  if (track < 0.) track += 360.0;

  float speed      = stateGetHorizontalSpeedNorm_f();
  float vspeed     = stateGetSpeedEnu_f()->z;

  float heading    = DegOfRad(stateGetNedToBodyEulers_f()->psi);
  if (heading < 0.) heading += 360.0;

  float pitch      = DegOfRad(stateGetNedToBodyEulers_f()->theta);
  float roll       = DegOfRad(stateGetNedToBodyEulers_f()->phi);

  char  company[]  = TO_ATC_COMPANY;
  char  model[]    = TO_ATC_MODEL;

  DOWNLINK_SEND_TO_ATC(DefaultChannel, DefaultDevice,
    strlen(flight),flight, strlen(callsign),callsign, &lat, &lon, &alt, &track, &speed, &vspeed,
    &heading, &pitch, &roll, strlen(tacode),tacode, strlen(company),company, strlen(model), model);
}
