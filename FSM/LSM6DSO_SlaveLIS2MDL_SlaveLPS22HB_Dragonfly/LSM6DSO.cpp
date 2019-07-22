/* 07/15/2019 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Dragonfly default), respectively, and it uses the Dragonfly STM32L476RE Breakout Board.
  
  https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/
  
  The LSM6DSO is a sensor hub with embedded accel and gyro, and a finite state machine with machine learning,
  here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "LSM6DSO.h"
#include "I2Cdev.h"
#include "LIS2MDL.h"
#include "LPS22HB.h"

LSM6DSO::LSM6DSO(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t LSM6DSO::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_WHO_AM_I);
  return c;
}


void LSM6DSO::sleepMode()
{
  uint8_t tempa = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL);
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, tempa & ~(0xF0) );
  uint8_t tempg = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL2_G);
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL2_G,  tempg & ~(0xF0) ); // power down gyro sensor
 // uint8_t tempg = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL4_C);
//  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL4_C,  tempg | 0x40 ); // enable gyro sleep mode for faster wake up
}


void LSM6DSO::wakeMode(uint8_t AODR, uint8_t GODR)
{
  uint8_t tempa = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL);
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, tempa | AODR << 4 );
  uint8_t tempg = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL2_G); 
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL2_G,  tempg | GODR << 4 );  // power up gyro sensor
//  uint8_t tempg = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL4_C);
//  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL4_C,  tempg & ~(0x40) ); // disable gyro sleep mode  
}


void LSM6DSO::idleLIS2MDL(uint8_t MODR)
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG);             // preserve contents of MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp & ~(0x04));          // Turn off I2C master
  delayMicroseconds(300);                                                                // wait 300 microseconds

  // Change Sensor Hub configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV0_ADD, (LIS2MDL_ADDRESS << 1));        // enable slave sensor write operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV0_SUBADD, LIS2MDL_CFG_REG_A);          // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_DATAWRITE_SLV0, 0x80 | MODR<<2 | 0x03);   // set idle mode bits (0 and 1)

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp );                   // Restore MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
}


void LSM6DSO::resumeLIS2MDL(uint8_t MODR)
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG);             // preserve contents of MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp & ~(0x04));          // Turn off I2C master
  delayMicroseconds(300);                                                                // wait 300 microseconds

  // Change Sensor Hub configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV0_ADD, (LIS2MDL_ADDRESS << 1));        // enable slave sensor write operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV0_SUBADD, LIS2MDL_CFG_REG_A);          // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_DATAWRITE_SLV0, 0x80 | MODR<<2);          // set continuous mode bits (0 and 1)

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp );                   // Restore MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
}


 void LSM6DSO::idleLPS22HB()
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG);             // preserve contents of MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp & ~(0x04));          // Turn off I2C master
  delayMicroseconds(300);                                                                // wait 200 microseconds

  // Change Sensor Hub configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV1_ADD, (LPS22H_ADDRESS << 1));         // enable slave sensor write operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV1_SUBADD, LPS22H_CTRL_REG1);           // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_DATAWRITE_SLV0, 0x00);                    // idle LPS22HB

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp );                   // Restore MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
}


void LSM6DSO::resumeLPS22HB(uint8_t PODR)
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG);             // preserve contents of MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp & ~(0x04));          // Turn off I2C master
  delayMicroseconds(300);                                                                // wait 300 microseconds

  // Change Sensor Hub configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV1_ADD, (LPS22H_ADDRESS << 1));         // enable slave sensor write operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV1_SUBADD, LPS22H_CTRL_REG1);           // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_DATAWRITE_SLV0, PODR << 4 | 0x08 | 0x02); // resume LPS22HB

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG,  temp );                   // Restore MASTER_CONFIG register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
}


void LSM6DSO::passthruMode()
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                    // enable sensor hub access
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG);              // preserve MASTER_CONFIG 
  // START_CONFIG = bit 5, PASSTHRU = bit 4, PULLUP_EN = bit 3, MASTER_ON = bit 2
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG, temp | 0x20);               // set START_CONFIG bit (bit 5) to 1 to disable sensor hub trigger
  delay(5);
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG, (temp | 0x20) & ~(0x04));                 // set MASTER_ON bit (bit 2) to 0
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG, (temp & ~(0x04)) & ~(0x20));              // set START_CONFIG bit (bit 5) to 0
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG, ((temp & ~(0x04)) & ~(0x20)) & ~(0x08));  // set PULLUP_EN bit (bit 3)to 0
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG, 0x10);                      // enable pass through  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                    // disable embedded functions
}


void LSM6DSO::masterMode()
{
  // MASTER_CONFIG
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  delay(100); // wait for all host I2C transactions to complete, 100 ms should be long enough
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG, 0x00);                     // disable passthrough mode
 
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
  uint8_t c = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL);                     // preserve accel configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, 0x00);                          // disable accel 
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL6_C, 0x80);                           // configure INT2 as input

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access

  // Configure LIS2MDL slave
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV0_ADD, (LIS2MDL_ADDRESS << 1) | 0x01); // enable slave sensor read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV0_SUBADD, LIS2MDL_OUTX_L_REG);         // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV0_CONFIG, 0x06);                       // read six bytes to read from slave 0

  // Configure LPS22HB slave
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV1_ADD, (LPS22H_ADDRESS << 1) | 0x01);  // enable slave sensor read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV1_SUBADD, LPS22H_PRESS_OUT_XL);        // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_SLV1_CONFIG, 0x05);                       // five bytes to read

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MASTER_CONFIG, 0x40 | 0x08 | 0x04 | 0x01);// set write once bit (bit 6) to 1, enable master mode (bit 2), internal pullups (bit 3), accel trigger drdy, two external sensors (bits 0/1)
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL6_C, 0x00);                           // unconfigure INT2 as input
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, c);                             // reset accel with previous configuration
}


float LSM6DSO::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
}

float LSM6DSO::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
  }
}


void LSM6DSO::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL3_C);
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset LSM6DSO
  delay(100); // Wait for all registers to reset
}


uint8_t LSM6DSO::DRstatus()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_STATUS_REG); // read ststus register
  return temp;
}


uint8_t LSM6DSO::ACTstatus()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_ALL_INT_SRC); // read status register
  return temp;
}


void LSM6DSO::ActivityDetect()
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_WAKE_UP_DUR, 0x08);        // set sleep duration at 8096 ODR (so about 5 seconds)
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_WAKE_UP_THS, 0x02);        // set wake threshold to 62.5 mg
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0);   // preserve interrupt existing configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0, temp & ~(0x20)); // set interrupt to report sleep status change
  temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2);           // preserve interrupt existing configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2, temp | 0xE0);    // power down gyro when asleep (set to 0x80 if notification only, no change in gyro accel mode, desired), enable wake/sleep interrupts
  temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);            // preserve interrupt existing routing
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x80);     // add sleep/wake interrupt routing to INT1
}


void LSM6DSO::singleTapDetect()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0);// preserve interrupt existing configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0, temp | 0x0E); // enable tap detection on X, Y, Z
   temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG1);       // preserve interrupt existing configuration  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG1, temp | 0x09); //  set x-axis threshold and axes priority
   temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2);       // preserve interrupt existing configuration  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2, temp | 0x89); // set y-axis threshold and enable interrupt
   temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_THS_6D);     // preserve interrupt existing configuration  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_THS_6D, temp | 0x09);   // set z-axis threshold 
   temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_INT_DUR2);       // preserve interrupt existing configuration  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_INT_DUR2, temp | 0x06); // set quiet and shock time windows
  temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);         // preserve interrupt existing routing
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x40);  // add single tap interrupt routing to INT1
}


void LSM6DSO::D6DOrientationDetect()
{
//  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0);    // preserve interrupt existing configuration
//  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG0, temp | 0x41);     // set latch mode with reset on read
   uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2);           // preserve interrupt existing configuration  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_CFG2, temp | 0x80);     // enable interrupt
   temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_THS_6D);         // preserve interrupt existing configuration  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_THS_6D, temp | 0x40);   // set 6D threshold to 60 degrees
  temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL8_XL);            // preserve accel existing configuration
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL8_XL, temp | 0x01);     // enable 6D functionality
  temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);             // preserve interrupt existing routing
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x04);      // add 6D interrupt routing to INT1
}


uint8_t LSM6DSO::EMBstatus()
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);         // enable embedded function access
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_STATUS); // read embedded function status register
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);         // disable embedded function access
  return temp;
}


uint8_t LSM6DSO::D6DSource()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_D6D_SRC); // read D6D source register
  return temp;
}


uint8_t LSM6DSO::TapSource()
{
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_TAP_SRC); // read tap source register
  return temp;
}


void LSM6DSO::TiltDetect()
{
  // embedded function-Tilt detect
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);       // enable embedded function access
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_EN_A, 0x10);         // enable tilt detection
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_INT1, 0x10);         // route tilt detect interrupt to INT1
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);       // disable embedded function access
  uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);       // preserve interrupt existing routing
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x02);        // add embedded functions interrupt routing to INT1
}


//
//  copied from FSM example in ST AN5273 
// (https://www.st.com/content/ccc/resource/technical/document/application_note/group1/6f/b8/c2/59/7e/00/43/c6/DM00572971/files/DM00572971.pdf/jcr:content/translations/en.DM00572971.pdf)
//
void LSM6DSO::FSMWristTiltDetect() {
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);                        // enable embedded function access
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_EN_B, 0x01);                          // enable FSM
    uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B);             // preserve default register contants
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B, temp | 0x01 << 3);         // select 26 Hz FSM ODR
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_ENABLE_A, 0x01);                           // Enable FSM engine 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_ENABLE_B, 0x00);                           // Disable all other FSM engines
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT1_A, 0x01);                             // Route FSM 1 interrupt to INT1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT1_B, 0x00);                             // Route no other interrupt to INT1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT2_A, 0x00);                             // Route no other interrupt to INT2
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT2_B, 0x00);                             // Route no other interrupt to INT2
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x40);                                // Enable page write

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x11);                               // Select page 1 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, 0x7A);                              // Select FSM_LC_TIMEOUT_L register on page 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_L (register auto increments)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_H
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // write 1 to FSM_PROGRAMS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // dummy write to skip to FSM_START_ADD_L register
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to FSM_START_ADDRESS_L
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x04);                             // write to FSM_START_ADDRESS_H

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL,   0x41);                             // Select page 4 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR,  0x00);                             // Select first address on program page 4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x51);                             // write to CONFIG_A (one threshold, one mask, one short timer)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x10);                             // write to SIZE (16 byte program)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xAE);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xB7);                             // write to THRESH1 MSB  (-0.4797 g in half precision floating point)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x80);                             // write to MASKA  (+x axis)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x10);                             // write to TIMER3

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x53);                             // write to GNTH1 | TI3
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x99);                             // write to OUTC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x50);                             // write to GNTH1 | NOP
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to  STOP

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL,   0x01);                             // Disable access to embedded advanced features
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x00);                                // Disable page write
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                        // disable embedded function access

    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);                                // preserve interrupt existing routing
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x02);                         // add embedded functions interrupt routing to INT1

    uint8_t tempa = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL);
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, tempa | 0x02 << 4 );                // set accel sample rate to 26 Hz
}


void LSM6DSO::FSMMotionDetect() {
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);                        // enable embedded function access
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_EN_B, 0x01);                          // enable FSM
    uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B);             // preserve default register contants
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B, temp | 0x01 << 3);         // select 26 Hz FSM ODR
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_ENABLE_A, 0x01);                           // Enable FSM engine 1
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT1_A,  0x01);                             // Route FSM 1 interrupt to INT1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x40);                                // Enable page write

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x11);                               // Select page 1 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, 0x7A);                              // Select FSM_LC_TIMEOUT_L register on page 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_L (register auto increments)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_H
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // write 1 to FSM_PROGRAMS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // dummy write to skip to FSM_START_ADD_L register
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to FSM_START_ADDRESS_L
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x04);                             // write to FSM_START_ADDRESS_H

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL,   0x41);                             // Select page 4 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR,  0x00);                             // Select first address on program page 4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x51);                             // write to CONFIG_A
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x14);                             // write to SIZE (20 byte program)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x66);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x3C);                             // write to THRESH1 MSB  (set to + 1.0996 g)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x02);                             // write to MASKA (+V)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x7D);                             // write to TIMER3 (125 samples at 26 Hz = 4.8 seconds)

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x05);                             // write to NOP | GNTH1 
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xC7);                             // write to UMSKIT Unmask interrupt generation when setting OUTS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x99);                             // write to OUTC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x33);                             // write to SRP Reset pointer to next address 
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x53);                             // write to GNTH1 | TI3
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x44);                             // write to CRP clear reset pointer to first program line
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xF5);                             // write to MSKIT Mask interrupt generation when setting OUTS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x22);                             // write to CONTREL Continues execution from reset pointer, resetting temporary mask
  
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x01);                               // Disable access to embedded advanced features
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x00);                                // Disable page write
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                        // disable embedded function access

    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);                                // preserve interrupt existing routing
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x02);                         // add embedded functions interrupt routing to INT1

    uint8_t tempa = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL);                      // set accel sample rate to 26 Hz
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, tempa | 0x02 << 4 );
}


void LSM6DSO::FSMShakeDetect() {
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);                        // enable embedded function access
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_EN_B, 0x01);                          // enable FSM
    uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B);             // preserve default register contants
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B, temp | 0x01 << 3);         // select 26 Hz FSM ODR
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_ENABLE_A, 0x01);                           // Enable FSM engine 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT1_A,  0x01);                            // Route FSM 1 interrupt to INT1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x40);                                // Enable page write

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x11);                               // Select page 1 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, 0x7A);                              // Select FSM_LC_TIMEOUT_L register on page 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_L (register auto increments)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_H
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // write 1 to FSM_PROGRAMS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // dummy write to skip to FSM_START_ADD_L register
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to FSM_START_ADDRESS_L
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x04);                             // write to FSM_START_ADDRESS_H

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL,   0x41);                             // Select page 4 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR,  0x00);                             // Select first address on program page 4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xC0 | 0x20 | 0x02);               // write to CONFIG_A (three thresholds, two masks, two short timers)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x1E);                             // write to SIZE (30 byte program)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x66);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x3E);                             // write to THRESH1 MSB  (set to + 1.6 g)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x66);                             // write to THRESH2 LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xBE);                             // write to THRESH2 MSB  (set to -1.6 g)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xCD);                             // write to THRESH3 LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x3C);                             // write to THRESH3 MSB  (set to + 1.2 g)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xC0);                             // write to MASKA (+X and -X)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x02);                             // write to MASKB (+V)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x10);                             // write to TIMER3 (16 samples at 26 Hz = 0.62 seconds)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x05);                             // write to TIMER4 ( 5 samples at 26 Hz = 0.19 seconds)

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x66);                             // write to SELMA Select MASKA and TMASKA as current mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xCC);                             // write to SELTHR1 Selects THRESH1 instead of THRESH3
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x35);                             // write to TI3 ! GNTH1  look for over/under/over threshold events == shaking
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x38);                             // write to TI3 ! LNTH2
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x35);                             // write to TI3 ! GNTH1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x77);                             // write to SELMB Select MASKB and TMASKB as current mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xDD);                             // write to SELTHR3 Selects THRESH3 instead of THRESH1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x03);                             // write to NOP | TI3
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x54);                             // write to GNTH1 | TI4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x22);                             // write to CONTREL Continues execution from reset pointer, resetting temporary mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to STOP
  
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x01);                               // Disable access to embedded advanced features
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x00);                                // Disable page write
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                        // disable embedded function access

    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);                                // preserve interrupt existing routing
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x02);                         // add embedded functions interrupt routing to INT1

    uint8_t tempa = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL);                      // set accel sample rate to 26 Hz
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, tempa | 0x02 << 4 );
}


void LSM6DSO::FSMGlanceDetect() {
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);                        // enable embedded function access
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_EN_B, 0x01);                          // enable FSM
    uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B);             // preserve default register contants
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B, temp | 0x01 << 3);         // select 26 Hz FSM ODR
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_ENABLE_A, 0x01);                           // Enable FSM engine 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT1_A,  0x01);                            // Route FSM 1 interrupt to INT1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x40);                                // Enable page write

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x11);                               // Select page 1 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, 0x7A);                              // Select FSM_LC_TIMEOUT_L register on page 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_L (register auto increments)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_H
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // write 1 to FSM_PROGRAMS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // dummy write to skip to FSM_START_ADD_L register
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to FSM_START_ADDRESS_L
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x04);                             // write to FSM_START_ADDRESS_H

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL,   0x41);                             // Select page 4 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR,  0x00);                             // Select first address on program page 4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x80 | 0x30 | 0x08 | 0x02);        // write to CONFIG_A (two thresholds, three masks, two long timers and two short timers)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to CONFIG_B
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x28);                             // write to SIZE (40 byte program)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to THRESH1 LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x38);                             // write to THRESH1 MSB  (set to + 0.5 g)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x66);                             // write to THRESH2 LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x2E);                             // write to THRESH2 MSB  (set to + 0.1 g)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x80);                             // write to MASKA (+X)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x20);                             // write to MASKB (+Y)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x08);                             // write to MASKC (+Z)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x02);                             // write to TIMER1 ( 2 samples at 26 Hz = 0.0772 seconds) LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             //  MSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x20);                             // write to TIMER2 (32 samples at 26 Hz = 1.23 seconds) LSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             //  MSB
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x0A);                             // write to TIMER3 (16 samples at 26 Hz = 0.39 seconds)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x03);                             // write to TIMER4 ( 3 samples at 26 Hz = 0.10 seconds)

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x23);                             // write to SINMUX Set input multiplexer
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // select gyroscope
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x66);                             // write to SELMA Select MASKA and TMASKA as current mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x16);                             // write to TI1 ! GNTH2   look for >thresh2 in TI1 seconds
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x23);                             // write to SINMUX Set input multiplexer
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x07);                             // select integrated gyroscope
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x25);                             // write to TI2 ! GNTH1   look for >thresh1 in TI2 seconds
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x23);                             // write to SINMUX Set input multiplexer
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // select accelerometer
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x03);                             // write to NOP | TI3
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x77);                             // write to SELMB Select MASKB and TMASKB as current mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x84);                             // write to LNTH2 | TI4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x88);                             // write to SELMC Select MASKC and TMASKC as current mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x84);                             // write to LNTH2 | TI4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x22);                             // write to CONTREL Continues execution from reset pointer, resetting temporary mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to STOP
  
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x01);                               // Disable access to embedded advanced features
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x00);                                // Disable page write
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                        // disable embedded function access

    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);                                // preserve interrupt existing routing
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x02);                         // add embedded functions interrupt routing to INT1

    uint8_t tempa = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL);                      // change accel and gyro sample rates to 26 Hz
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, tempa | 0x02 << 4 );
    uint8_t tempg = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL2_G); 
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL2_G,  tempg | 0x02 << 4 );   
}


void LSM6DSO::FSMMagZeroCrossingDetect() {
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);                        // enable embedded function access
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_EN_B, 0x01);                          // enable FSM
    uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B);             // preserve default register contants
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_EMB_FUNC_ODR_CFG_B, temp | 0x11 << 3);         // select 26 Hz FSM ODR
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_ENABLE_A, 0x01);                           // Enable FSM engine 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_ENABLE_B, 0x00);                           // Disable all other FSM engines
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT1_A, 0x01);                             // Route FSM 1 interrupt to INT1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT1_B, 0x00);                             // Route no other interrupt to INT1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT2_A, 0x00);                             // Route no other interrupt to INT2
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_INT2_B, 0x00);                             // Route no other interrupt to INT2
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x40);                                // Enable page write

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x11);                               // Select page 1 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, 0x7A);                              // Select FSM_LC_TIMEOUT_L register on page 1
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_L (register auto increments)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write 0 to FSM_LONG_COUNTER_H
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // write 1 to FSM_PROGRAMS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x01);                             // dummy write to skip to FSM_START_ADD_L register
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to FSM_START_ADDRESS_L
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x04);                             // write to FSM_START_ADDRESS_H

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL,   0x41);                             // Select page 4 (bit 0 must always be 1)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR,  0x00);                             // Select first address on program page 4
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x11);                             // write to CONFIG_A (one mask, one short timer)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x10);                             // write to CONFIG_B (enable zero crossing detection)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x12);                             // write to SIZE (18 byte program)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to SETTINGS
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to RESET POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to PROGRAM POINTER
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x08);                             // write to MASKA  (+z axis)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TMASKA
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to TC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x10);                             // write to TIMER3 (0.6 seconds)
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to PAS

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x23);                             // write to SINMUX Set input multiplexer
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x02);                             // select calibrated magnetometer
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xD3);                             // write to PZC | TI3
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x99);                             // write to OUTC
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0xD0);                             // write to PZC | NOP
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x22);                             // write to CONTREL Continues execution from reset pointer, resetting temporary mask
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x00);                             // write to STOP

    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL,   0x01);                             // Disable access to embedded advanced features
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x00);                                // Disable page write
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                        // disable embedded function access

    temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG);                                // preserve interrupt existing routing
    _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_MD1_CFG, temp | 0x02);                         // add embedded functions interrupt routing to INT1
}
uint8_t LSM6DSO::FSMstatus()
 {
   uint8_t temp = _i2c_bus->readByte(LSM6DSO_ADDRESS, LSM6DSO_FSM_STATUS_A_MAINPAGE); // read FSM status register
  return temp;
 }

 
void LSM6DSO::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL1_XL, AODR << 4 | Ascale << 2); // set accel full scale and sample rate

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL2_G, GODR << 4 | Gscale << 1); // set gyro full scale and sample rate

  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL3_C, 0x40 | 0x04);
  // by default, interrupts active HIGH, push pull, little endian data
  // (can be changed by writing to bits 5, 4, and 1, resp to above register)

  //  mask data ready until filter settle complete (bit 3)
//  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL4_C, 0x08);
  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL9_XL, 0x02);  // disable I3C MIPI interface

  // interrupt handling
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_COUNTER_BDR_REG1, 0x80); // data ready pulsed mode
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_INT1_CTRL, 0x03);        // enable gyro and accel data ready interrupts on INT1
}


void LSM6DSO::readMagData(int16_t * destination)
{
  uint8_t rawData[8];  // x/y/z mag register data stored here
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  _i2c_bus->readBytes(LSM6DSO_ADDRESS, LSM6DSO_SENSOR_HUB_1, 6, &rawData[0]);              // Read the 6 raw data registers into data array
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access

  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}


int32_t LSM6DSO::readBaroData()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  _i2c_bus->readBytes(LSM6DSO_ADDRESS, LSM6DSO_SENSOR_HUB_7, 3, &rawData[0]);  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
  return (int32_t) ((int32_t) rawData[2] << 16 | (int32_t) rawData[1] << 8 | rawData[0]);
}


int16_t LSM6DSO::readBaroTemp()
{
    uint8_t rawData[2];  // 16-bit pressure register data stored here
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x40);                   // enable sensor hub access
  _i2c_bus->readBytes(LSM6DSO_ADDRESS, LSM6DSO_SENSOR_HUB_10, 2, &rawData[0]);  
  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);                   // disable sensor hub access
  return (int16_t)((int16_t) rawData[1] << 8 | rawData[0]);
}


void LSM6DSO::selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readAccelGyroData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond
  readAccelGyroData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL5_C, 0x03); // negative accel self test
  delay(100); // let accel respond
  readAccelGyroData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL5_C, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  readAccelGyroData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL5_C, 0x0C); // negative gyro self test
  delay(100); // let gyro respond
  readAccelGyroData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_CTRL5_C, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");
  Serial.println(" ");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  Serial.println(" ");
  delay(2000);
}


void LSM6DSO::AGoffsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(10000);

  for (uint8_t ii = 0; ii < 128; ii++)
  {
    readAccelGyroData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1] * _gRes / 128.0f;
  dest1[1] = sum[2] * _gRes / 128.0f;
  dest1[2] = sum[3] * _gRes / 128.0f;
  dest2[0] = sum[4] * _aRes / 128.0f;
  dest2[1] = sum[5] * _aRes / 128.0f;
  dest2[2] = sum[6] * _aRes / 128.0f;

  if (dest2[0] > 0.75f)  {
    dest2[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest2[0] < -0.75f) {
    dest2[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest2[1] > 0.75f)  {
    dest2[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest2[1] < -0.75f) {
    dest2[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest2[2] > 0.75f)  {
    dest2[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest2[2] < -0.75f) {
    dest2[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}


void LSM6DSO::MagoffsetBias(float * dest1, float * dest2)
{
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  float _mRes = 0.0015f;
  
  Serial.println("Calculate mag offset bias: move all around to sample the complete response surface!");
  delay(4000);

  for (int ii = 0; ii < 2000; ii++)
  {
    readMagData(mag_temp);
       for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(12);
  }

  _mRes = 0.0015f; // fixed sensitivity
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0] * _mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1] * _mRes;   
    dest1[2] = (float) mag_bias[2] * _mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

   // Store mag calibration in Embedded Advanced Features Page 0 for later use by FSM and Machine Learning engines
   // Offset Bias
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x80);   // enable embedded function access
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x40);           // select page write
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_SEL, 0x01);          // select page 0

   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_OFFX_L);                       // write low byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, (FloattoHalf(dest1[0])) & 0x00FF);        // write calibration data
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_OFFX_H);                       // write high byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, ((FloattoHalf(dest1[0])) & 0xFF00) >> 8); // write calibration data

   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_OFFY_L);                       // write low byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, (FloattoHalf(dest1[1])) & 0x00FF);        // write calibration data
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_OFFY_H);                       // write high byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, ((FloattoHalf(dest1[1])) & 0xFF00) >> 8); // write calibration data

   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_OFFZ_L);                       // write low byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, (FloattoHalf(dest1[2])) & 0x00FF);        // write calibration data
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_OFFZ_H);                       // write high byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, ((FloattoHalf(dest1[2])) & 0xFF00) >> 8); // write calibration data

   // Scale factor
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_SI_XX_L);                       // write low byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, (FloattoHalf(dest2[0])) & 0x00FF);         // write calibration data
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_SI_XX_H);                       // write high byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, ((FloattoHalf(dest2[0])) & 0xFF00) >> 8);  // write calibration data

   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_SI_YY_L);                       // write low byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, (FloattoHalf(dest2[1])) & 0x00FF);         // write calibration data
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_SI_YY_H);                       // write high byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, ((FloattoHalf(dest2[1])) & 0xFF00) >> 8);  // write calibration data

   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_SI_ZZ_L);                       // write low byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, (FloattoHalf(dest2[2])) & 0x00FF);         // write calibration data
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_SI_ZZ_H);                       // write high byte to mag calibration
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, ((FloattoHalf(dest2[2])) & 0xFF00) >> 8);  // write calibration data

   // Align accel/gyro and mag axes
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_ADDR, LSM6DSO_MAG_CFG_A);                         // write to coordinate rotation register
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_VALUE, 0x10         );                            // accel/gyro y = mag -y

   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_PAGE_RW, 0x00);               // deselect page write
   _i2c_bus->writeByte(LSM6DSO_ADDRESS, LSM6DSO_FUNC_CFG_ACCESS, 0x00);       // disable embedded function access
  
   Serial.println("Mag Calibration done!");
}


void LSM6DSO::readAccelGyroData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(LSM6DSO_ADDRESS, LSM6DSO_OUT_TEMP_L, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = (int16_t)((int16_t)rawData[1] << 8)  | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((int16_t)rawData[3] << 8)  | rawData[2] ;
  destination[2] = (int16_t)((int16_t)rawData[5] << 8)  | rawData[4] ;
  destination[3] = (int16_t)((int16_t)rawData[7] << 8)  | rawData[6] ;
  destination[4] = (int16_t)((int16_t)rawData[9] << 8)  | rawData[8] ;
  destination[5] = (int16_t)((int16_t)rawData[11] << 8) | rawData[10] ;
  destination[6] = (int16_t)((int16_t)rawData[13] << 8) | rawData[12] ;
}


uint16_t LSM6DSO::FloattoHalf(float f)
{
  uint32_t x = *((uint32_t*)&f);
  uint16_t h = ((x>>16)&0x8000)|((((x&0x7F800000)-0x38000000)>>13)&0x7C00)|((x>>13)&0x03FF);
  return h;
}

