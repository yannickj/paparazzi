/** @file modules/datalink/osdlink.h
 */

#include "std.h"

#ifndef DATALINK_OSDLINK_H
#define DATALINK_OSDLINK_H

#define OSD_CMD_CAPTURE_STOP 0
#define OSD_CMD_CAPTURE_RUN 1
#define OSD_CMD_MAPPING_END 2

extern uint8_t osd_cmd;

void osdlink_init(void);
void osdlink_periodic(void);
void osdlink_event(void);

#endif // DATALINK_OSDLINK_H
