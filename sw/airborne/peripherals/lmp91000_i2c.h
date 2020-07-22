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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Based on the work of Linnes Lab https://github.com/LinnesLab/LMP91000
 */

/**
 * @file modules/sensors/lmp91000_i2c.h
 * @brief 
 *
 */

#ifndef LMP91000_I2C_H
#define LMP91000_I2C_H

#include "mcu_periph/i2c.h"


#define LMP91000_TEMP_INTERCEPT        1555
#define LMP91000_TEMPSLOPE        -8
#define LMP91000_I2C_ADDR        0X48

#define LMP91000_STATUS_REG        0x00    /* Read only status register */
#define LMP91000_LOCK_REG      0x01    /* Protection Register */
#define LMP91000_TIACN_REG      0x10    /* TIA Control Register */
#define LMP91000_REFCN_REG      0x11    /* Reference Control Register*/
#define LMP91000_MODECN_REG      0x12    /* Mode Control Register */

#define LMP91000_READY      0x01
#define LMP91000_NOT_READY      0x00

#define LMP91000_TIA_GAIN_EXT      0x00 //default
#define LMP91000_TIA_GAIN_2P75K      0x04
#define LMP91000_TIA_GAIN_3P5K      0x08
#define LMP91000_TIA_GAIN_7K      0x0C
#define LMP91000_TIA_GAIN_14K      0x10
#define LMP91000_TIA_GAIN_35K      0x14
#define LMP91000_TIA_GAIN_120K      0x18
#define LMP91000_TIA_GAIN_350K      0x1C

#define LMP91000_RLOAD_10OHM      0X00
#define LMP91000_RLOAD_33OHM      0X01
#define LMP91000_RLOAD_50OHM      0X02
#define LMP91000_RLOAD_100OHM      0X03 //default

#define LMP91000_REF_SOURCE_INT      0x00 //default
#define LMP91000_REF_SOURCE_EXT      0x80

#define LMP91000_INT_Z_20PCT      0x00
#define LMP91000_INT_Z_50PCT      0x20//default
#define LMP91000_INT_Z_67PCT      0x40
#define LMP91000_INT_Z_BYPASS      0x60

#define LMP91000_BIAS_SIGN_NEG      0x00 //default
#define LMP91000_BIAS_SIGN_POS      0x10

#define LMP91000_BIAS_0PCT      0x00 //default
#define LMP91000_BIAS_1PCT      0x01
#define LMP91000_BIAS_2PCT      0x02
#define LMP91000_BIAS_4PCT      0x03
#define LMP91000_BIAS_6PCT      0x04
#define LMP91000_BIAS_8PCT      0x05
#define LMP91000_BIAS_10PCT      0x06
#define LMP91000_BIAS_12PCT      0x07
#define LMP91000_BIAS_14PCT      0x08
#define LMP91000_BIAS_16PCT      0x09
#define LMP91000_BIAS_18PCT      0x0A
#define LMP91000_BIAS_20PCT      0x0B
#define LMP91000_BIAS_22PCT      0x0C
#define LMP91000_BIAS_24PCT      0x0D

#define LMP91000_FET_SHORT_DISABLED      0x00 //default
#define LMP91000_FET_SHORT_ENABLED      0x80
#define LMP91000_OP_MODE_DEEP_SLEEP      0x00 //default
#define LMP91000_OP_MODE_GALVANIC      0x01
#define LMP91000_OP_MODE_STANDBY      0x02
#define LMP91000_OP_MODE_AMPEROMETRIC      0x03
#define LMP91000_OP_MODE_TIA_OFF      0x06
#define LMP91000_OP_MODE_TIA_ON      0x07

#define LMP91000_WRITE_LOCK      0x01 //default
#define LMP91000_WRITE_UNLOCK      0x00

#define LMP91000_NOT_PRESENT      0xA8    // arbitrary library status code

#define LMP91000_NUM_TIA_BIAS        14

extern const double LMP91000_TIA_GAIN[];
extern const double LMP91000_TIA_BIAS[];
extern const double LMP91000_TIA_ZERO[];

// TODO define these registers, remove #define LMP91000_DEFAULT_REGISTER after
#define LMP91000_DEFAULT_REGISTER 0
#define LMP91000_ALL                         LMP91000_DEFAULT_REGISTER

enum Lmp91000Status {
  LMP91000_STATUS_UNINIT,
  LMP91000_STATUS_GET_CALIB,
  LMP91000_STATUS_CONFIGURE,
  LMP91000_STATUS_READ_DATA
};

struct lmp91000_reg_calib_data {
  uint16_t par_t1;
};

struct Lmp91000_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Lmp91000Status status;           ///< state machine status
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  struct lmp91000_reg_calib_data calib; ///< calibration data
// #if (LMP91000_COMPENSATION == LMP91000_DOUBLE_PRECISION_COMPENSATION) || ( LMP91000_COMPENSATION == LMP91000_SINGLE_PRECISION_COMPENSATION)
  // struct k30_quantized_calib_data quant_calib; ///< quantized calibration data
// #endif
  uint32_t raw_co;                 ///< uncompensated co concentration
  uint32_t raw_temperature;         ///< uncompensated temperature
  float co;                        ///< co concentration in ppm
  float temperature;                ///< temperature in deg Celcius
  uint8_t MENB;                     ///< IO pin for enabling and disabling I2C commands
  uint8_t gain;
  uint8_t zero;
};

// extern void lmp91000_i2c_read_eeprom_calib(struct Lmp91000_I2c *lmp91000);
extern void lmp91000_i2c_init(struct Lmp91000_I2c *lmp91000, struct i2c_periph *i2c_p, uint8_t addr);
extern void lmp91000_i2c_periodic(struct Lmp91000_I2c *lmp91000);
extern void lmp91000_i2c_event(struct Lmp91000_I2c *lmp91000);

/*
//sets and gets MENB pin for enabling and disabling I2C commands
void lmp91000_setMENB(uint8_t pin);
uint8_t lmp91000_getMENB();
*/

//sets and gets pin for reading output of temperature sensor
void lmp91000_setTempSensor(uint8_t pin);
// uint8_t lmp91000_getTempSensor(); Not found ænt

//reads and writes to LMP91000 via I2C
void lmp91000_write(uint8_t reg, uint8_t data);
uint8_t lmp91000_read(uint8_t reg);

//enables and disables LMP91000 for I2C commands
//default state is not ready
/*
void lmp91000_enable();
void lmp91000_disable();
*/
uint8_t lmp91000_isReady(void);

//locks and unlocks the transimpedance amplifier
//and reference control registers for editing
//default state is locked (read-only)
void lmp91000_lock(void);
void lmp91000_unlock(void);
uint8_t lmp91000_isLocked(void);

//sets the gain of the transimpedance amplifier
void lmp91000_setGain(struct Lmp91000_I2c *lmp91000, uint8_t gain);
double lmp91000_getGain(struct Lmp91000_I2c *lmp91000);

//sets the load for compensating voltage differences
//between working and reference electrodes
void lmp91000_setRLoad(uint8_t load);

//sets the source for the bias voltage for the
//electrochemical cell
void lmp91000_setRefSource(uint8_t source);
void lmp91000_setIntRefSource(void);
void lmp91000_setExtRefSource(void);

//sets reference voltage for transimpedance amplifier
void lmp91000_setIntZ(struct Lmp91000_I2c *lmp91000, uint8_t intZ);
double lmp91000_getIntZ(struct Lmp91000_I2c *lmp91000);

//sets bias voltage for electrochemical cell
void lmp91000_setBiasSign(uint8_t sign);
void lmp91000_setNegBias(void);
void lmp91000_setPosBias(void);
void lmp91000_setBias(uint8_t bias);
void lmp91000_setBias_sign(uint8_t bias, signed char sign);

//enable and disable FET for deep sleep mode
void lmp91000_setFET(uint8_t selection);
void lmp91000_disableFET(void);
void lmp91000_enableFET(void);

//set operating modes for the LMP91000
void lmp91000_setMode(uint8_t mode);
void lmp91000_sleep(void);
void lmp91000_setTwoLead(void);
void lmp91000_standby(void);
void lmp91000_setThreeLead(void);
void lmp91000_measureCell(void);
void lmp91000_getTemp(void);
double lmp91000_getTemp_adc(uint8_t sensor, double adc_ref, uint8_t adc_bits);

//reading the output of the LMP91000
uint16_t lmp91000_getOutput(uint8_t sensor);
double lmp91000_getVoltage(uint16_t adcVal, double adc_ref, uint8_t adc_bits);
double lmp91000_getCurrent(struct Lmp91000_I2c *lmp91000, uint16_t adcVal, double adc_ref, uint8_t adc_bits);
double lmp91000_getCurrentExtGain(struct Lmp91000_I2c *lmp91000, uint16_t adcVal, double adc_ref, uint8_t adc_bits, double extGain);

#endif /* LMP91000_I2C_H */