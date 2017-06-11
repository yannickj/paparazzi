#include "modules/gpslogsd/gpslogsd.h"
#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/uart.h"

struct uart_periph *gpsdev;

/*
 * Ublox receveiver should be configured with ubx protocol
 * and send raw messages :
 * RXM-RAWX
 * RXM-SFRBX
*/

/**********************************************************************/
void gpslogsd_init(void)
{
  gpsdev =  &(GPSLOGSD_DEV);
}

/**********************************************************************/
void gpslogsd_periodic(void)
{
}

/*********** EVENT ***********************************************************/
void gpslogsd_event(void)
{
  if (gpsdownLogFile == -1) return;
  while (uart_char_available(gpsdev)) 
    sdLogWriteByte(gpsdownLogFile, uart_getch(gpsdev));
}
