#include "modules/uart_log/uart_log.h"
#include "modules/loggers/sdlog_chibios.h"
#include "mcu_periph/uart.h"

struct uart_periph *uartdev;


/**********************************************************************/
void uart_log_init(void)
{
  uartdev =  &(UARTLOG_DEV);
}

/**********************************************************************/
void uart_log_periodic(void)
{
}

/*********** EVENT ***********************************************************/
void uart_log_event(void)
{
  if (uartLogFile == -1) return;
  while (uart_char_available(uartdev)) 
    sdLogWriteByte(uartLogFile, uart_getch(uartdev));
}
