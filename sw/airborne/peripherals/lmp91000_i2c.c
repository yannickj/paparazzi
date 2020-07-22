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
 * @file modules/sensors/lmp91000_i2c.c
 * @brief 
 *
 */

#include "peripherals/lmp91000_i2c.h"
#include "mcu_periph/sys_time_arch.h"


// definition of array
const double LMP91000_TIA_GAIN[] = {2750,3500,7000,14000,35000,120000,350000};
const double LMP91000_TIA_BIAS[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.22, 0.24};
const double LMP91000_TIA_ZERO[] = {0.2, 0.5, 0.67};

// TODO local function declarations

static void parse_sensor_data(struct Lmp91000_I2c *lmp91000);
static double compensate_temperature(struct Lmp91000_I2c *lmp91000);
static double compensate_co(struct Lmp91000_I2c *lmp91000);

/**
 * init function
 */
void lmp91000_i2c_init(struct Lmp91000_I2c *lmp91000, struct i2c_periph *i2c_periph, uint8_t addr){
  /* set i2c_peripheral */
  lmp91000->i2c_p = i2c_periph;

  /* slave address */
  lmp91000->i2c_trans.slave_addr = addr;
  /* set initial status: Done */
  lmp91000->i2c_trans.status = I2CTransDone;

  lmp91000->data_available = false;
  lmp91000->initialized = false;
  lmp91000->status = LMP91000_STATUS_UNINIT;
}

void lmp91000_i2c_periodic(struct Lmp91000_I2c *lmp91000){
  if (lmp91000->i2c_trans.status != I2CTransDone) {
    return; // transaction not finished
  }

  switch (lmp91000->status) {
    case LMP91000_STATUS_UNINIT:
      lmp91000->data_available = false;
      lmp91000->initialized = false;
      lmp91000->status = LMP91000_STATUS_CONFIGURE;
      break;

    // case LMP91000_STATUS_GET_CALIB:
    //   break;

    case LMP91000_STATUS_CONFIGURE:
      // // From datasheet, recommended config for drone usecase:
      // // osrs_p = 8, osrs_t = 1
      // // IIR filter = 2 (note: this one doesn't exist...)
      // // ODR = 50
      // lmp91000->i2c_trans.buf[0] = LMP91000_PWR_CTRL_ADDR;
      // lmp91000->i2c_trans.buf[1] = LMP91000_ALL | LMP91000_NORMAL_MODE << 4;
      // lmp91000->i2c_trans.buf[2] = LMP91000_OSR_ADDR;
      // lmp91000->i2c_trans.buf[3] = LMP91000_OVERSAMPLING_8X | LMP91000_NO_OVERSAMPLING << 3;
      // lmp91000->i2c_trans.buf[4] = LMP91000_ODR_ADDR;
      // lmp91000->i2c_trans.buf[5] = LMP91000_ODR_50_HZ;
      // lmp91000->i2c_trans.buf[6] = LMP91000_CONFIG_ADDR;
      // lmp91000->i2c_trans.buf[7] = LMP91000_IIR_FILTER_COEFF_3 << 1;
      // i2c_transmit(lmp91000->i2c_p, &lmp91000->i2c_trans, lmp91000->i2c_trans.slave_addr, 8);
      break;

    case LMP91000_STATUS_READ_DATA:
      /* read data */
      /*
      lmp91000->i2c_trans.buf[0] = LMP91000_SENS_STATUS_REG_ADDR;
      i2c_transceive(lmp91000->i2c_p, &lmp91000->i2c_trans, lmp91000->i2c_trans.slave_addr, 1, LMP91000_C02_AND_T_HEADER_DATA_LEN);
      */
      break;

    default:
      break;
  }
}

void lmp91000_i2c_event(struct Lmp91000_I2c *lmp91000){
  if (lmp91000->i2c_trans.status == I2CTransSuccess) {
    switch (lmp91000->status) {
      // case LMP91000_STATUS_GET_CALIB:
      //   // compute calib
      //   parse_calib_data(lmp91000);
      //   lmp91000->status = LMP91000_STATUS_CONFIGURE;
      //   break;

      case LMP91000_STATUS_CONFIGURE:
        // nothing else to do, start reading
        lmp91000->status = LMP91000_STATUS_READ_DATA;
        lmp91000->initialized = true;
        break;

      case LMP91000_STATUS_READ_DATA:
        // check status byte
        if (lmp91000->i2c_trans.buf[0] & (LMP91000_ALL << 5)) {
          // parse sensor data, compensate temperature first, then co concentration
          parse_sensor_data(lmp91000);
          compensate_temperature(lmp91000);
          compensate_co(lmp91000);
          lmp91000->data_available = true;
        }
        break;

      default:
        break;
    }
    lmp91000->i2c_trans.status = I2CTransDone;
  } else if (lmp91000->i2c_trans.status == I2CTransFailed) {
    /* try again */
    if (!lmp91000->initialized) {
      lmp91000->status = LMP91000_STATUS_UNINIT;
    }
    lmp91000->i2c_trans.status = I2CTransDone;
  }
}

static void parse_sensor_data(struct Lmp91000_I2c *lmp91000){
    (void) lmp91000; //rm warning
}

/**
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(struct Lmp91000_I2c *lmp91000)
{
  // return lmp91000->quant_calib.t_lin;
  (void) lmp91000; // rm warning
  double comp_t;
  return comp_t;
}

/**
 * @brief This internal API is used to compensate the raw co concentration data and
 * return the compensated co concentration data in double data type.
 */
static double compensate_co(struct Lmp91000_I2c *lmp91000)
{
  (void) lmp91000; //rm warning
  /* Variable to store the compensated co2 concentration */
  double comp_press;

  return comp_press;
}

/* ænt ----------------------------------------------------
   below code from  Linnes Lab
   AUTHOR:	Orlando S. Hoilett
   EMAIL:     ohoilett@purdue.edu */

/* useless (only one lmp91000 used, MENB = GND) ænt
//void setMENB(uint8_t pin)
//Sets the MENB I/O pin and initializes pin with "pinMode" function.
void lmp91000_setMENB(uint8_t pin)
{
    MENB = pin;
    pinMode(MENB, OUTPUT);
}


//uint8_t lmp91000_getMENB() const
//Returns the I/O pin for controlling the MENB (module enable).
uint8_t lmp91000_getMENB() 
{
    return MENB;
}

//void LMP91000::enable() const
//ENABLES the LMP91000 for I2C operations. Please consult page 3, "Section 5 Pin
//Configurations and Functions" for more information.
//
//The device is active low.
void LMP91000::enable() const
{
    digitalWrite(MENB, LOW);
}


//void LMP91000::disable() const
//DISABLES the LMP91000 for I2C operations. Please consult page 3, "Section 5
//Pin Configurations and Functions" for more information.
//
//The device is active low.
void LMP91000::disable() const
{
    digitalWrite(MENB, HIGH);
}
*/

//void lmp91000_write(uint8_t reg, uint8_t data) const
//@param        reg: register to write to
//@param		data: data that will be written to register
//
//First ensures the device is enabled for I2C commands using the "enable()"
//method. Writes to the LMP91000 I2C device using the I2C protocol for Arduino
//https://www.arduino.cc/en/Reference/WireWrite
//Please consult page 20, Section 7.5.1 I2C Interface and 7.5.2 Write and Read
//Operation in the datsheet for more information.
void lmp91000_write(uint8_t reg, uint8_t data) 
{
    (void) reg; //rm warning
    (void) data; //rm warning
    /*TODO change to transmit ænt*/
    // enable();
    /*
    Wire.beginTransmission(LMP91000_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
    */
}


//uint8_t lmp91000_read(uint8_t reg) const
//@param        reg: register to read from
//
//First ensures the device is enabled for I2C commands using the "enable()"
//method. Reads from the LMP91000 I2C device using the I2C protocol for Arduino.
//https://www.arduino.cc/en/Reference/WireRead
//
//Please consult page 20, "Section 7.5.1 I2C Interface" and "7.5.2 Write and Read
//Operation" in the datsheet for more information.
//
//The device has must be written to first before a read operation can be performed.
uint8_t lmp91000_read(uint8_t reg) 
{
    (void) reg; //rm warning
    /*TODO change to recieve ænt*/
    uint8_t data = 0;
    
    // enable();
    /*
    Wire.beginTransmission(LMP91000_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    Wire.requestFrom(LMP91000_I2C_ADDRESS, 0x01);
    while(Wire.available()){
        data = Wire.read();
    }
    */
    
    return data;
}





//uint8_t lmp91000_isReady() const
//@return       whether or not the device is ready.
//
//Reads the status register (0x00) of the LMP91000 to determine whether or not
//the device is ready to accept I2C commands.
//
//Please consult page 21, "Section 7.6.1 STATUS -- Status Register (Address
//0x00)" of the datasheet for more information.
//
//Default state is not ready.
uint8_t lmp91000_isReady(void)
{
    uint8_t r = lmp91000_read(LMP91000_STATUS_REG)==LMP91000_READY;
    return r;
}


//uint8_t isLocked() const
//@return       whether or not the TIACN and REFCN is locked for writing
//
////Reads the lock register (0x01) of the LMP91000 to determine whether or not
//the TIACN and REFCN are "write-enabled" or "read-only."
//
//Please consult pages 21 and 22, "Section 7.6.2 LOCK -- Protection Register
//(Address 0x01)" for more information.
//
//Deafult state is "read-only" mode.
uint8_t lmp91000_isLocked(void)
{
    // return bitRead(read(LMP91000_LOCK_REG),0)==LMP91000_WRITE_LOCK;
    return (!(lmp91000_read(LMP91000_LOCK_REG) & 1)) == LMP91000_WRITE_LOCK; // read the first bit
}

//from vicatcu
//void lmp91000_lock() const
//
//Writes to the lock register (0x01) of the LMP9100 to set the TIACN and REFCN
//registers to "read-only."
//
//Please consult pages 21 and 22, "Section 7.6.2 LOCK -- Protection Register
//(Address 0x01)" for more information.
//
//Default state is "read-only" mode.
void lmp91000_lock(void)
{
    lmp91000_write(LMP91000_LOCK_REG, LMP91000_WRITE_LOCK);
}

//from vicatcu
//void lmp91000_unlock() const
//
//Writes to the lock register (0x01) of the LMP9100 to set the TIACN and REFCN
//registers to "write" mode.
//
//Please consult pages 21 and 22, "Section 7.6.2 LOCK -- Protection Register
//(Address 0x01)" for more information.
//
//Default state is "read-only" mode.
void lmp91000_unlock(void)
{
    lmp91000_write(LMP91000_LOCK_REG, LMP91000_WRITE_UNLOCK);
}


//void lmp91000_setGain(uint8_t gain) const
//@param            gain: the gain to be set to
//
//param - value - gain resistor
//0 - 000 - External resistor
//1 - 001 - 2.75 kOhm
//2 - 010 - 3.5 kOhm
//3 - 011 - 7 kOhm
//4 - 100 - 14 kOhm
//5 - 101 - 35 kOhm
//6 - 110 - 120 kOhm
//7 - 111 - 350 kOhm
//
//Sets the transimpedance amplifier gain. First reads the register to ensure
//that the other bits are not affected. The 3 LSBs of "gain" parameter is
//written to the 2nd, 3rd, and 4th bit of the TIACN register.
//
//Please consult page 14 "7.3.1.1 Transimpedance Amplifier" and page 22 "Section
//7.6.3 TIACN -- TIA Control Register (Address 0x10)" of the datasheet for more
//information.
void lmp91000_setGain(struct Lmp91000_I2c *lmp91000, uint8_t user_gain)
{
    lmp91000->gain = user_gain;
    
    lmp91000_unlock();
    uint8_t data = lmp91000_read(LMP91000_TIACN_REG);
    data &= ~(7 << 2); //clears bits 2-4
    data |= (user_gain << 2); //writes to bits 2-4
    lmp91000_write(LMP91000_TIACN_REG, data);
}


double lmp91000_getGain(struct Lmp91000_I2c *lmp91000)
{
    if (lmp91000->gain== 0) return lmp91000->gain;
    else return LMP91000_TIA_GAIN[lmp91000->gain];
}

//void lmp91000_setRLoad(uint8_t load) const
//@param            load: the internal load resistor to select
//
//param - value - RLoad
//0 - 00 - 10 Ohm
//1 - 01 - 33 Ohm
//2 - 10 - 50 Ohm
//3 - 11 - 100 Ohm
//
//Sets the internal RLOAD selection resistor. First reads the register to ensure
//that the other bits are not affected. The 2 LSBs of "load" parameter is
//written to the 0th and 1st bit of the TIACN register.
//
//Please consult page 14 "7.3.1.1 Transimpedance Amplifier" and page 22 "Section
//7.6.3 TIACN -- TIA Control Register (Address 0x10)" of the datasheet for more
//information.
void lmp91000_setRLoad(uint8_t load) 
{
    lmp91000_unlock();
    uint8_t data = lmp91000_read(LMP91000_TIACN_REG);
    data &= ~3; //clears 0th and 1st bits
    data |= load; //writes to 0th and 1st bits
    lmp91000_write(LMP91000_TIACN_REG, data);
}

//void lmp91000_setRefSource(uint8_t source) const
//@param            source: external vs. internal
//
//param - result
//0 - internal reference
//1 - external reference
//
//Sets the voltage reference source of the LMP91000 to an internal reference or
//an external reference.
//
//Please consult page 22, "Section 7.6.4 REFCN -- Reference Control Register
//(Address 0x11)" of the datasheet for more information.
void lmp91000_setRefSource(uint8_t source) 
{
    if (source == 0) lmp91000_setIntRefSource();
    else lmp91000_setExtRefSource();

}


//void lmp91000_setIntRefSource() const
//
//Unlocks the REFCN register for "write" mode. First reads the register to
//ensure that the other bits are not affected. Writes a "0" to the 7th bit of
//the REFCN register.
//
//Sets the voltage reference source to supply voltage (Vdd).
//
//Please consult page 22, "Section 7.6.4 REFCN -- Reference Control Register
//(Address 0x11)" of the datasheet for more information.
void lmp91000_setIntRefSource(void)
{
    lmp91000_unlock(); //unlocks the REFCN register for "write" mode
    uint8_t data = lmp91000_read(LMP91000_REFCN_REG);
    data &= ~(1 << 7); //clears the 7th bit
    lmp91000_write(LMP91000_REFCN_REG, data);
}


//void lmp91000_setExtRefSource() const
//
//Unlocks the REFCN register for "write" mode. First reads the register to
//ensure that the other bits are not affected. Writes a "1" to the 7th bit of
//the REFCN register.
//
//Sets the reference source of the LMP91000 to an external reference provided at
//the Vref pin.
//
//Please consult page 22, "Section 7.6.4 REFCN -- Reference Control Register
//(Address 0x11)" of the datasheet for more information.
void lmp91000_setExtRefSource(void)
{
    lmp91000_unlock(); //unlocks the REFCN register for "write" mode
    uint8_t data = lmp91000_read(LMP91000_REFCN_REG);
    data |= (1 << 7); //writes a "1" to the 7th bit
    lmp91000_write(LMP91000_REFCN_REG, data);
}


//void lmp91000_setIntZ(uint8_t intZ) const
//@param            intZ: the internal zero selection
//
//param - value - result
//0 - 00 - 20%
//1 - 01 - 50%
//2 - 10 - 67%
//3 - 11 - bypassed
//
//Unlocks the REFCN register for "write" mode. First reads the register to
//ensure that the other bits are not affected. Writes to the 5th and 6th bits
//of the REFCN register.
//
//Sets the internal zero of the device, particularly the transimpedance
//amplifier.
//
//Please consult page 22, "Section 7.6.4 REFCN -- Reference Control Register
//(Address 0x11)" of the datasheet for more information.
void lmp91000_setIntZ(struct Lmp91000_I2c *lmp91000,uint8_t intZ)
{
    lmp91000->zero = intZ;
    
    lmp91000_unlock(); //unlocks the REFCN register for "write" mode
    uint8_t data = lmp91000_read(LMP91000_REFCN_REG);
    data &= ~(3 << 5);
    data |= (intZ << 5);
    lmp91000_write(LMP91000_REFCN_REG, data);
}

double lmp91000_getIntZ(struct Lmp91000_I2c *lmp91000)
{
    return LMP91000_TIA_ZERO[lmp91000->zero];
}


//void lmp91000_setBiasSign(uint8_t sign) const
//0 = negative
//1 = positive
void lmp91000_setBiasSign(uint8_t sign) 
{
    if (sign == 0) lmp91000_setNegBias();
    else lmp91000_setPosBias();
}

//void lmp91000_setNegBias() const
void lmp91000_setNegBias(void)
{
    lmp91000_unlock();
    uint8_t data = lmp91000_read(LMP91000_REFCN_REG);
    data &= ~(1 << 4); //clear bit
    lmp91000_write(LMP91000_REFCN_REG, data);
}

//void lmp91000_setPosBias() const
void lmp91000_setPosBias(void)
{
    lmp91000_unlock();
    uint8_t data = lmp91000_read(LMP91000_REFCN_REG);
    data |= (1 << 4);
    lmp91000_write(LMP91000_REFCN_REG, data);
}

//void lmp91000_setBias(uint8_t bias) const
void lmp91000_setBias(uint8_t bias) 
{
    lmp91000_unlock();
    uint8_t data = lmp91000_read(LMP91000_REFCN_REG);
    data &= ~(0x0F); //clear the first four bits in order to bit Or in the next step
    data |= bias;
    lmp91000_write(LMP91000_REFCN_REG, data);
}


//void lmp91000_setBias(uint8_t bias) const
//sign				0 is negative and 1 is positive
//
void lmp91000_setBias_sign(uint8_t bias, signed char sign) 
{
	if(sign > 0) sign = 1;
	else sign = 0;
	sign = (uint8_t)sign;
	
	if(bias > 13) bias = 0;
	
	
	lmp91000_unlock();
	uint8_t data = lmp91000_read(LMP91000_REFCN_REG);
	data &= ~(0x1F); //clear the first five bits in order to bit Or in the next step
	data |= bias;
	data |= ((sign << 4) | bias);
	lmp91000_write(LMP91000_REFCN_REG, data);
}


//void lmp91000_setFET(uint8_t selection) const
void lmp91000_setFET(uint8_t selection) 
{
    if (selection == 0) lmp91000_disableFET();
    else lmp91000_enableFET();
}

//void lmp91000_disableFET() const
void lmp91000_disableFET(void)
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data &= ~(1 << 7);
    lmp91000_write(LMP91000_MODECN_REG, data);
}

//void lmp91000_enableFET() const
void lmp91000_enableFET(void)
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data |= (1 << 7);
    lmp91000_write(LMP91000_MODECN_REG, data);
}

//void lmp91000_setMode(uint8_t mode) const
void lmp91000_setMode(uint8_t mode) 
{
    if (mode == 0) lmp91000_sleep();
    else if (mode == 1) lmp91000_setTwoLead();
    else if (mode == 2) lmp91000_standby();
    else if (mode == 3) lmp91000_setThreeLead();
    else if (mode == 4) lmp91000_measureCell();
    else if (mode == 5) lmp91000_getTemp();
	else {}; //some error
}

//void lmp91000_sleep const
//
//Sets the 3 LSBs of the Mode Control Register (0x12) to 0.
//
//Places the LMP91000 in deep sleep state for power conservation. The LMP91000
//consumes 0.6 uA of current in deep sleep mode.
//
//Please see page 19 Section, 7.4 Device Functional Modes and page 23, Section
//7.6.5 MODECN -- Mode Control Register (Address 0x12) of the datasheet for more
//information.
void lmp91000_sleep(void)
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data &= ~(0x07);
    lmp91000_write(LMP91000_MODECN_REG, data);
}


//void lmp91000_setTwoLead() const
//Sets the first three bits of the Mode Control Register to 001. This enables
//the LMP91000 for 2-electrode potentiometric measurements.
void lmp91000_setTwoLead(void)
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data &= ~(0x07);
    data |= (0x01);
    lmp91000_write(LMP91000_MODECN_REG, data);
}


//void lmp91000_standby() const
//
//Sets the 3 LSBs of the Mode Control Register (0x12) to 010.
//
//Places the device in standby() mode which allows quick warm-up in between tests
//
//Please see page 19-20, Section 7.4 Device Functional Modes and page 23 Section
//7.6.5 MODECN -- Mode Control Register (Address 0x12) of the datasheet for
//more information.
void lmp91000_standby(void)
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data &= ~(0x07);
    data |= (0x02);
    lmp91000_write(LMP91000_MODECN_REG, data);
}

//void lmp91000_setThreeLead() const
//Sets the first three bits of the Mode Control Register to 011. This enables
//the LMP91000 for 3-electrode potentiometric measurements.
void lmp91000_setThreeLead(void)
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data &= ~(0x07);
    data |= (0x03);
    lmp91000_write(LMP91000_MODECN_REG, data);
}

//void lmp91000_measureCell() const
//
void lmp91000_measureCell(void)
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data &= ~(0x07); //clears the first three bits
    data |= (0x06); // put 110 at the end of data --> temperature measurement OFF, to regain VOUT = measure_cell ænt
    lmp91000_write(LMP91000_MODECN_REG, data);
}

//void lmp91000_getTemp() const
void lmp91000_getTemp(void) 
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data |= (0x07);
    lmp91000_write(LMP91000_MODECN_REG, data);
}


//double lmp91000_getTemp(uint8_t sensor, double adc_ref, uint8_t adc_bits) const
//returns               temperatue in degrees Celsius
//
//Measures temperature by setting bits 0, 1, and 2 of the Mode Control Register
//to 1. This sets the transimpedance amplifier of the LMP91000 ON and sends
//the output of the internal temperature sensor to the VOUT pin of the LMP91000.
double lmp91000_getTemp_adc(uint8_t sensor, double adc_ref, uint8_t adc_bits) 
{
    uint8_t data = lmp91000_read(LMP91000_MODECN_REG);
    data |= (0x07);
    lmp91000_write(LMP91000_MODECN_REG, data);
    sys_time_msleep(100);   
    return (lmp91000_getVoltage(sensor, adc_ref, adc_bits)-LMP91000_TEMP_INTERCEPT)/LMP91000_TEMPSLOPE;
}

//ænt change to use ADC
//uint16_t MiniStat::getOutput(uint8_t sensor) const
//
//@param            sensor: the analog in pin of the LMP91000 is connected to
//
//@return           the voltage output of the LMP91000 in bits
//
//Uses analogRead() return the output of the LMP91000.
uint16_t lmp91000_getOutput(uint8_t sensor) 
{
    // return analogRead(sensor);
    (void) sensor; //rm warning
    return 0;
}


//double MiniStat::getVoltage(uint16_t adcVal, double adc_ref, uint8_t adc_bits) const
//
//@param            adcVal: value returned by the analog-to-digital converter of
//                          the microcontroller used to control the LMP91000
//
//@param            adc_ref: voltage reference of the analog-to-digtal converter
//                          of the microcontroller
//
//@param            adc_bits: number of bits of the analog-to-digital converter
//                          of the microcontroller
//
//@return           the voltage output of the LMP91000
//
//This method calculates the voltage at the output of the LMP91000 by multiplying
//by the refernece voltage of the analog-to-digital converter and dividing by
//the bit resolution of the analog-to-digital converter.
double lmp91000_getVoltage(uint16_t adcVal, double adc_ref, uint8_t adc_bits) 
{
    return (adcVal*adc_ref)/(pow(2,adc_bits)-1);
}
 

//double MiniStat::getCurrent(uint16_t adcVal, double adc_ref, uint8_t adc_bits) const
//
//@param            adcVal: value returned by the analog-to-digital converter of
//                          the microcontroller used to control the LMP91000
//
//@param            adc_ref: voltage reference of the analog-to-digtal converter
//                          of the microcontroller
//
//@param            adc_bits: number of bits of the analog-to-digital converter
//                          of the microcontroller
//
//@return           the current at the working electrode
//
//This method calculates the current at the working electrode by reading in the
//voltage at the output of LMP91000 and dividing by the value of the gain resistor.
double lmp91000_getCurrent(struct Lmp91000_I2c *lmp91000, uint16_t adcVal, double adc_ref, uint8_t adc_bits) 
{
    return (lmp91000_getVoltage(adcVal, adc_ref, adc_bits) - (adc_ref*LMP91000_TIA_ZERO[lmp91000->zero]))/LMP91000_TIA_GAIN[lmp91000->gain-1];
}


//double MiniStat::getCurrent(uint16_t adcVal, double adc_ref, uint8_t adc_bits,
//                              double extGain) const
//
//@param            adcVal: value returned by the analog-to-digital converter of
//                          the microcontroller used to control the LMP91000
//
//@param            adc_ref: voltage reference of the analog-to-digtal converter
//                          of the microcontroller
//
//@param            adc_bits: number of bits of the analog-to-digital converter
//                          of the microcontroller
//
//@param            extGain: value of external gain resistor
//
//@return           the current at the working electrode
//
//This method calculates the current at the working electrode by reading in the
//voltage at the output of LMP91000 and dividing by the value of the external
//gain resistor.
double lmp91000_getCurrentExtGain(struct Lmp91000_I2c *lmp91000, uint16_t adcVal, double adc_ref, uint8_t adc_bits,
                            double extGain) 
{
    return (lmp91000_getVoltage(adcVal, adc_ref, adc_bits) - (adc_ref*LMP91000_TIA_ZERO[lmp91000->zero]))/extGain;
}
