/** @file modules/datalink/osdlink.c
 */

#include "modules/datalink/osdlink.h"

#include "mcu_periph/uart.h"

#include "state.h"

#include "subsystems/datalink/downlink.h"

#include "subsystems/gps.h"

#include "navigation.h"

#define __OsdLink(dev, _x) dev##_x
#define _OsdLink(dev, _x)  __OsdLink(dev, _x)
#define OsdLink(_x) _OsdLink(OSD_LINK, _x)

uint8_t osd_cmd;

struct datatypeOut {
  int32_t  lat;   ///< in degrees*1e7
  int32_t  lon;   ///< in degrees*1e7
  int32_t  alt;   ///< in millimeters above WGS84 reference ellipsoid
  uint32_t pacc;  ///< accuraccy in cm
  float    x;     ///< in m
  float    y;     ///< in m
  float    z;     ///< in m
  int16_t  phi;   ///< in decideg
  int16_t  theta; ///< in decideg
  int16_t  psi;   ///< in decideg
  uint8_t  cmd;   ///< command
};

struct datatypeIn {
  uint32_t  cpt;
  int32_t   lat;   ///< in degrees*1e7
  int32_t   lon;   ///< in degrees*1e7
  int32_t   alt;   ///< in millimeters above WGS84 reference ellipsoid
};

bool_t moveWptOnce = TRUE;

struct datatypeIn dataIn;

void osdlink_init(void)
{
  memset(&dataIn, 0, sizeof(dataIn));

  osd_cmd = OSD_CMD_CAPTURE_STOP;
}


void osdlink_periodic(void)
{
  struct LlaCoor_i lla;
  struct EnuCoor_f enu;
  struct datatypeOut data;

  // Build OSD data

  data.lat = gps.lla_pos.lat;
  data.lon = gps.lla_pos.lon;
  data.alt = gps.lla_pos.alt;

  data. pacc = gps.pacc;

  enu = *stateGetPositionEnu_f();
  data.x = enu.x;
  data.y = enu.y;
  data.z = enu.z;

  data.phi   = DegOfRad(stateGetNedToBodyEulers_f()->phi * 10.0f);
  data.theta = DegOfRad(stateGetNedToBodyEulers_f()->theta * 10.0f);
  data.psi   = DegOfRad(stateGetNedToBodyEulers_f()->psi * 10.0f);

  data.cmd = osd_cmd;

  const uint8_t lg=2+sizeof(data)+1;
  uint8_t buff[lg];
  int cpt=0;

  buff[cpt]=0xFF; cpt=cpt+1;         // Start
  buff[cpt]=lg;   cpt=cpt+1;         // Length

  memcpy(&(buff[cpt]),&data,sizeof(data)); cpt=cpt+sizeof(data);

  buff[cpt]=0xFE;  cpt=cpt+1;       // End

  // Send OSD data to daughter board

  int i=0;
  while (i<cpt) {
    OsdLink(Transmit(buff[i]));
    i++;
  }

  // Send OSD message to ground

  pprz_msg_send_OSD(&(DefaultChannel).trans_tx, &(DefaultDevice).device, AC_ID, &dataIn.cpt, &dataIn.lat, &dataIn.lon, &dataIn.alt);


  // Move waypoint (onboard updated and requested message to ground)

  if((moveWptOnce) && (dataIn.cpt==1)) {
    lla.lat = dataIn.lat;
    lla.lon = dataIn.lon;
    nav_move_waypoint_lla(20, &lla); // WP_p1
    moveWptOnce = FALSE;
  }

}

#define OSD_PARSE_WAIT_SYNC 0
#define OSD_PARSE_GOT_SYNC 1
#define OSD_PARSE_GOT_LENGTH 2
#define OSD_PARSE_GOT_PAYLOAD 3

#define OSD_PAYLOAD_SIZE 64

static int parse_state = OSD_PARSE_WAIT_SYNC;
static int osd_length = 0;
static int osd_current = 0;
static bool_t osd_payload_valid = FALSE;
static uint8_t buffIn[OSD_PAYLOAD_SIZE];


static void parse_osd(uint8_t c)
{
  switch (parse_state) {
    case OSD_PARSE_WAIT_SYNC:
      if (c == 0xFF) {
        parse_state = OSD_PARSE_GOT_SYNC;
        osd_length = 0;
        osd_current = 0;
      }
      break;
    case OSD_PARSE_GOT_SYNC:
      osd_length = c - 3;
      parse_state = OSD_PARSE_GOT_LENGTH;
      break;
    case OSD_PARSE_GOT_LENGTH:
      osd_payload_valid = FALSE;
      buffIn[osd_current++] = c;
      if (osd_current >= osd_length) {
        parse_state = OSD_PARSE_GOT_PAYLOAD;
      }
      break;
    case OSD_PARSE_GOT_PAYLOAD:
      if (c == 0xFE) {
        osd_payload_valid = TRUE;
      }
      parse_state = OSD_PARSE_WAIT_SYNC;
      break;
    default:
      parse_state = OSD_PARSE_WAIT_SYNC;
  }
}

void osdlink_event(void)
{
  while (OsdLink(ChAvailable())) {
    parse_osd(OsdLink(Getch()));
    if (osd_payload_valid) {
      memcpy(&dataIn, &buffIn, sizeof(dataIn));
      break;
    }
  }
}
