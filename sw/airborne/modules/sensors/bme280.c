/*
 * Copyright (C)
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING. If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/sensors/bme280.c
 * @brief Bosch-Sensortech BME280 temperature-pressure-humidity sensor interface
 *
 * This reads temperature, pressure and humidity from the BME280 sensor through I2C.
 */

#include "bme280.h"
// #include <stdio.h>

// not useful until now
// #include "subsystems/abi.h"
// #include "pprzlink/messages.h"
// #include "subsystems/datalink/downlink.h"

/** default slave address */
#ifndef BME280_SLAVE_ADDR
#define BME280_SLAVE_ADDR BME280_I2C_ADDR
#endif

struct Bme280_I2c_Spi bme280;

// #if LOG_INERIS
// static bool log_bme280_started;
// static void bme280_log_data_ascii(void);
// #endif

void bme280_init(void)
{
  bme280_i2c_spi_init(&bme280, &BME280_I2C_DEV, BME280_SLAVE_ADDR);
  // #if LOG_INERIS
  // log_bme280_started = false;
  // #endif
}

void bme280_periodic(void)
{ 
  bme280_i2c_spi_periodic(&bme280);
}

void bme280_event(void)
{
  bme280_i2c_spi_event(&bme280);

  if (bme280.data_available) {
    print_char_uart("Bme 280 ----\n", 13);
    print_uint32_uart(bme280.raw_pressure, "raw p:  ", 8);
    print_float_uart(bme280.pressure,"p (hPa):", 8);
    print_uint32_uart(bme280.raw_temperature, "raw t:  ", 8); //TODO temperature is int32_t
    print_float_uart(bme280.temperature,"t (dgC):", 8);
    print_uint32_uart(bme280.raw_humidity, "raw h:  ", 8);
    print_float_uart(bme280.humidity,"h (%):  ", 8);

    print_uint16_uart(bme280.debug_flag, "bdg:    ", 8);
    print_char_uart("------------\n", 13);

    bme280.data_available = false;
  }
}
