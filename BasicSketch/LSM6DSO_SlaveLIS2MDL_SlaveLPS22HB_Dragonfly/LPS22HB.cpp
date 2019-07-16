/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Dragonfly default), respectively, and it uses the Dragonfly STM32L476RE Breakout Board.
  
  https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/
  
  The LPS22HB is a low power barometer.

  Library may be used freely and without limit with attribution.

*/

#include "LPS22HB.h"
#include "I2Cdev.h"

LPS22H::LPS22H(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}

uint8_t LPS22H::getChipID()
{
  // Read the WHO_AM_I register of the altimeter this is a good test of communication
  uint8_t temp = _i2c_bus->readByte(LPS22H_ADDRESS, LPS22H_WHOAMI);  // Read WHO_AM_I register for LPS22H
  return temp;
}


void LPS22H::reset()
{
 _i2c_bus->writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG2, 0x04);  // Reset LPS22H
}


uint8_t LPS22H::status()
{
  // Read the status register of the altimeter  
  uint8_t temp = _i2c_bus->readByte(LPS22H_ADDRESS, LPS22H_STATUS);   
  return temp;
}


void LPS22H::sleep()
{
  // set sample rate by setting bits 6:4 
  // enable low-pass filter by setting bit 3 to one
  // bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
  // make sure data not updated during read by setting block data udate (bit 1) to 1
    _i2c_bus->writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG1, 0x08 | 0x02);  
}


void LPS22H::wake(uint8_t PODR)
{
  // set sample rate by setting bits 6:4 
  // enable low-pass filter by setting bit 3 to one
  // bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
  // make sure data not updated during read by setting block data udate (bit 1) to 1
    _i2c_bus->writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG1, PODR << 4 | 0x08 | 0x02);  
}


void LPS22H::Init(uint8_t PODR)
{
  // set sample rate by setting bits 6:4 
  // enable low-pass filter by setting bit 3 to one
  // bit 2 == 0 means bandwidth is odr/9, bit 2 == 1 means bandwidth is odr/20
  // make sure data not updated during read by setting block data udate (bit 1) to 1
    _i2c_bus->writeByte(LPS22H_ADDRESS, LPS22H_CTRL_REG1, PODR << 4 | 0x08 | 0x02);  
}

