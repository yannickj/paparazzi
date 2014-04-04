#pragma once



/*!
  \def SATCOM_RX_POLLING_INTERVAL
  * intervall in seconds, in which in absence of tx message, rx message presence
  * will be tested (it needs the modem to be waked, so the less this value,
  * more the electricity consumption
  */

#ifndef SATCOM_DEFAULT_POLLING_INTERVAL
#define SATCOM_DEFAULT_POLLING_INTERVAL 60
#endif

typedef struct {
  uint32_t pollingInterval;
  bool_t   msgPending;
} Satcom;


/**
 * @brief   Initializes a satCom channel.
 * @satcomInit
 */
void satcomInit (void);


/**
 * @brief   push a binary message to the outcomming queue
 * @details A worker thread launched by satcomInit use AT command to
 *          communicate with the satcom iridium modem
 *
 * @param[in] buffer    pointer to the binary message to send
 * @param[in] size      size of the message
 *
 * @return              number of bytes sent, or negative error index
 * @retval ERROR_MAILBOX_FULL         if max number of pending message exceed max
 * @retval ERROR_CIRCULAR_BUFFER_FULL if size of all queued messages exceed max
 * @retval ERROR_MAILBOX_FAIL         if internal error occurs

 * @satcomSendBuffer
 */
int32_t satcomSendBuffer (const uint8_t * const buffer, const uint8_t size);



/**
 * @brief   pop a binary message from the incomming queue
 * @details A worker thread launched by satcomInit use AT command to
 *          communicate with the satcom iridium modem
 *
 * @param[in]  buffer    pointer to the binary message to get
 * @param[in]  bufferSize      size of the buffer
 *
 * @return              number of bytes received, or negative error index
 * @retval 0            if no pending message
 * @retval ERROR_MAILBOX_FAIL if internal error occurs

 * @satcomReceiveBuffer
 */
int32_t satcomReceiveBuffer (uint8_t * const buffer, const uint8_t bufferSize);


/**
 * @brief   set the time in second between two polling for received iridium message
 * @details if time is more then 60 seconds, the iridium modem is set asleep to
 *          save power
 *
 * @param[in]  polling interval time in seconds
 *
 * @satcomSetPollingInterval
 */
void    satcomSetPollingInterval (const uint32_t pollingInterval);
