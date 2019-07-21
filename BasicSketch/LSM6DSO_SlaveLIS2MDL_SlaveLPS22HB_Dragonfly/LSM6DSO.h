/* 07/15/2019 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Dragonfly default), respectively, and it uses the Dragonfly STM32L476RE Breakout Board.

  https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/
  
  The LSM6DSO is a sensor hub with embedded accel and gyro, and a finite state machine with machine learning,
  here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef LSM6DSO_h
#define LSM6DSO_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

/* LSM6DSO registers
  http://www.st.com/content/ccc/resource/technical/document/datasheet/76/27/cf/88/c5/03/42/6b/DM00218116.pdf/files/DM00218116.pdf/jcr:content/translations/en.DM00218116.pdf
*/
#define LSM6DSO_FUNC_CFG_ACCESS           0x01
#define LSM6DSO_PIN_CTRL                  0x02
#define LSM6DSO_S4S_TPH_L                 0x04
#define LSM6DSO_S4S_TPH_H                 0x05
#define LSM6DSO_S4S_RR                    0x06
#define LSM6DSO_FIFO_CTRL1                0x07
#define LSM6DSO_FIFO_CTRL2                0x08
#define LSM6DSO_FIFO_CTRL3                0x09
#define LSM6DSO_FIFO_CTRL4                0x0A
#define LSM6DSO_COUNTER_BDR_REG1          0x0B
#define LSM6DSO_COUNTER_BDR_REG2          0x0C
#define LSM6DSO_INT1_CTRL                 0x0D
#define LSM6DSO_INT2_CTRL                 0x0E
#define LSM6DSO_WHO_AM_I                  0x0F  // should be 0x6C
#define LSM6DSO_CTRL1_XL                  0x10
#define LSM6DSO_CTRL2_G                   0x11
#define LSM6DSO_CTRL3_C                   0x12
#define LSM6DSO_CTRL4_C                   0x13
#define LSM6DSO_CTRL5_C                   0x14
#define LSM6DSO_CTRL6_C                   0x15
#define LSM6DSO_CTRL7_G                   0x16
#define LSM6DSO_CTRL8_XL                  0x17
#define LSM6DSO_CTRL9_XL                  0x18
#define LSM6DSO_CTRL10_C                  0x19
#define LSM6DSO_ALL_INT_SRC               0x1A
#define LSM6DSO_WAKE_UP_SRC               0x1B
#define LSM6DSO_TAP_SRC                   0x1C
#define LSM6DSO_D6D_SRC                   0x1D
#define LSM6DSO_STATUS_REG                0x1E
#define LSM6DSO_OUT_TEMP_L                0x20
#define LSM6DSO_OUT_TEMP_H                0x21
#define LSM6DSO_OUTX_L_G                  0x22
#define LSM6DSO_OUTX_H_G                  0x23
#define LSM6DSO_OUTY_L_G                  0x24
#define LSM6DSO_OUTY_H_G                  0x25
#define LSM6DSO_OUTZ_L_G                  0x26
#define LSM6DSO_OUTZ_H_G                  0x27
#define LSM6DSO_OUTX_L_XL                 0x28
#define LSM6DSO_OUTX_H_XL                 0x29
#define LSM6DSO_OUTY_L_XL                 0x2A
#define LSM6DSO_OUTY_H_XL                 0x2B
#define LSM6DSO_OUTZ_L_XL                 0x2C
#define LSM6DSO_OUTZ_H_XL                 0x2D
#define LSM6DSO_EMB_FUNC_STATUS_MAINPAGE  0x35
#define LSM6DSO_FSM_STATUS_A_MAINPAGE     0x36
#define LSM6DSO_FSM_STATUS_B_MAINPAGE     0x37
#define LSM6DSO_MLC_STATUS_MAINPAGE       0x38
#define LSM6DSO_STATUS_MASTER_MAINPAGE    0x39
#define LSM6DSO_FIFO_STATUS1              0x3A
#define LSM6DSO_FIFO_STATUS2              0x3B
#define LSM6DSO_TIMESTAMP0                0x40
#define LSM6DSO_TIMESTAMP1                0x41
#define LSM6DSO_TIMESTAMP2                0x42
#define LSM6DSO_TIMESTAMP3                0x43
#define LSM6DSO_UI_STATUS_REG_OIS         0x49
#define LSM6DSO_UI_OUTX_L_G_OIS           0x4A
#define LSM6DSO_UI_OUTX_H_G_OIS           0x4B
#define LSM6DSO_UI_OUTY_L_G_OIS           0x4C
#define LSM6DSO_UI_OUTY_H_G_OIS           0x4D
#define LSM6DSO_UI_OUTZ_L_G_OIS           0x4E
#define LSM6DSO_UI_OUTZ_H_G_OIS           0x4F
#define LSM6DSO_UI_OUTX_L_A_OIS           0x50
#define LSM6DSO_UI_OUTX_H_A_OIS           0x51
#define LSM6DSO_UI_OUTY_L_A_OIS           0x52
#define LSM6DSO_UI_OUTY_H_A_OIS           0x53
#define LSM6DSO_UI_OUTZ_L_A_OIS           0x54
#define LSM6DSO_UI_OUTZ_H_A_OIS           0x55
#define LSM6DSO_TAP_CFG0                  0x56
#define LSM6DSO_TAP_CFG1                  0x57
#define LSM6DSO_TAP_CFG2                  0x58
#define LSM6DSO_TAP_THS_6D                0x59
#define LSM6DSO_INT_DUR2                  0x5A
#define LSM6DSO_WAKE_UP_THS               0x5B
#define LSM6DSO_WAKE_UP_DUR               0x5C
#define LSM6DSO_FREE_FALL                 0x5D
#define LSM6DSO_MD1_CFG                   0x5E
#define LSM6DSO_MD2_CFG                   0x5F
#define LSM6DSO_S4S_ST_CMD_CODE           0x60
#define LSM6DSO_S4S_DT_REG                0x61
#define LSM6DSO_I3C_BUS_AVB               0x62
#define LSM6DSO_INTERNAL_FREQ_FINE        0x63
#define LSM6DSO_INT_OIS                   0x6F
#define LSM6DSO_CTRL1_OIS                 0x70
#define LSM6DSO_CTRL2_OIS                 0x71
#define LSM6DSO_CTRL3_OIS                 0x72
#define LSM6DSO_X_OFS_USR                 0x73
#define LSM6DSO_Y_OFS_USR                 0x74
#define LSM6DSO_Z_OFS_USR                 0x75
#define LSM6DSO_FIFO_DATA_OUT_TAG         0x78
#define LSM6DSO_FIFO_DATA_OUT_X_L         0x79
#define LSM6DSO_FIFO_DATA_OUT_X_H         0x7A
#define LSM6DSO_FIFO_DATA_OUT_Y_L         0x7B
#define LSM6DSO_FIFO_DATA_OUT_Y_H         0x7C
#define LSM6DSO_FIFO_DATA_OUT_Z_L         0x7D
#define LSM6DSO_FIFO_DATA_OUT_Z_H         0x7E

// Embedded functions
#define LSM6DSO_PAGE_SEL                  0x02
#define LSM6DSO_EMB_FUNC_EN_A             0x04
#define LSM6DSO_EMB_FUNC_EN_B             0x05
#define LSM6DSO_PAGE_ADDRESS              0x08
#define LSM6DSO_PAGE_VALUE                0x09
#define LSM6DSO_EMB_FUNC_INT1             0x0A
#define LSM6DSO_FSM_INT1_A                0x0B
#define LSM6DSO_FSM_INT1_B                0x0C
#define LSM6DSO_EMB_FUNC_INT2             0x0E
#define LSM6DSO_FSM_INT2_A                0x0F
#define LSM6DSO_FSM_INT2_B                0x10
#define LSM6DSO_EMB_FUNC_STATUS           0x12
#define LSM6DSO_FSM_STATUS_A              0x13
#define LSM6DSO_FSM_STATUS_B              0x14
#define LSM6DSO_PAGE_RW                   0x17
#define LSM6DSO_EMB_FUNC_FIFO_CFG         0x44
#define LSM6DSO_FSM_ENABLE_A              0x46
#define LSM6DSO_FSM_ENABLE_B              0x47
#define LSM6DSO_FSM_LONG_COUNTER_L        0x48
#define LSM6DSO_FSM_LONG_COUNTER_H        0x49
#define LSM6DSO_FSM_LONG_COUNTER_CLEAR    0x4A
#define LSM6DSO_FSM_OUTS1                 0x4C
#define LSM6DSO_FSM_OUTS2                 0x4D
#define LSM6DSO_FSM_OUTS3                 0x4E
#define LSM6DSO_FSM_OUTS4                 0x4F
#define LSM6DSO_FSM_OUTS5                 0x50
#define LSM6DSO_FSM_OUTS6                 0x51
#define LSM6DSO_FSM_OUTS7                 0x52
#define LSM6DSO_FSM_OUTS8                 0x53
#define LSM6DSO_FSM_OUTS9                 0x54
#define LSM6DSO_FSM_OUTS10                0x55
#define LSM6DSO_FSM_OUTS11                0x56
#define LSM6DSO_FSM_OUTS12                0x57
#define LSM6DSO_FSM_OUTS13                0x58
#define LSM6DSO_FSM_OUTS14                0x59
#define LSM6DSO_FSM_OUTS15                0x5A
#define LSM6DSO_FSM_OUTS16                0x5B
#define LSM6DSO_EMB_FUNC_ODR_CFG_B        0x5F
#define LSM6DSO_EMB_FUNC_ODR_CFG_C        0x60
#define LSM6DSO_STEP_COUNTER_L            0x62
#define LSM6DSO_STEP_COUNTER_H            0x63
#define LSM6DSO_EMB_FUNC_SRC              0x64
#define LSM6DSO_EMB_FUNC_INIT_A           0x66
#define LSM6DSO_EMB_FUNC_INIT_B           0x67
#define LSM6DSO_MLC0_SRC                  0x70
#define LSM6DSO_MLC1_SRC                  0x71
#define LSM6DSO_MLC2_SRC                  0x72
#define LSM6DSO_MLC3_SRC                  0x73
#define LSM6DSO_MLC4_SRC                  0x74
#define LSM6DSO_MLC5_SRC                  0x75
#define LSM6DSO_MLC6_SRC                  0x76
#define LSM6DSO_MLC7_SRC                  0x77

// Page 0
#define LSM6DSO_MAG_SENSITIVITY_L         0xBA
#define LSM6DSO_MAG_SENSITIVITY_H         0xBB
#define LSM6DSO_MAG_OFFX_L                0xC0
#define LSM6DSO_MAG_OFFX_H                0xC1
#define LSM6DSO_MAG_OFFY_L                0xC2
#define LSM6DSO_MAG_OFFY_H                0xC3
#define LSM6DSO_MAG_OFFZ_L                0xC4
#define LSM6DSO_MAG_OFFZ_H                0xC5
#define LSM6DSO_MAG_SI_XX_L               0xC6
#define LSM6DSO_MAG_SI_XX_H               0xC7
#define LSM6DSO_MAG_SI_XY_L               0xC8
#define LSM6DSO_MAG_SI_XY_H               0xC9
#define LSM6DSO_MAG_SI_XZ_L               0xCA
#define LSM6DSO_MAG_SI_XZ_H               0xCB
#define LSM6DSO_MAG_SI_YY_L               0xCC
#define LSM6DSO_MAG_SI_YY_H               0xCD
#define LSM6DSO_MAG_SI_YZ_L               0xCE
#define LSM6DSO_MAG_SI_YZ_H               0xCF
#define LSM6DSO_MAG_SI_ZZ_L               0xD0
#define LSM6DSO_MAG_SI_ZZ_H               0xD1
#define LSM6DSO_MAG_CFG_A                 0xD4
#define LSM6DSO_MAG_CFG_B                 0xD5

// Page 1
#define LSM6DSO_FSM_LC_TIMEOUT_L          0x7A
#define LSM6DSO_FSM_LC_TIMEOUT_H          0x7B
#define LSM6DSO_FSM_PROGRAMS              0x7C
#define LSM6DSO_FSM_START_ADD_L           0x7E
#define LSM6DSO_FSM_START_ADD_H           0x7F
#define LSM6DSO_PEDO_CMD_REG              0x83
#define LSM6DSO_PEDO_DEB_STEPS_CONF       0x84
#define LSM6DSO_PEDO_SC_DELTAT_L          0xD0
#define LSM6DSO_PEDO_SC_DELTAT_H          0xD1
#define LSM6DSO_MLC_MAG_SENSITIVITY_L     0xE8
#define LSM6DSO_MLC_MAG_SENSITIVITY_H     0xE9

// Sensor Hub Registers
#define LSM6DSO_SENSOR_HUB_1              0x02
#define LSM6DSO_SENSOR_HUB_2              0x03
#define LSM6DSO_SENSOR_HUB_3              0x04
#define LSM6DSO_SENSOR_HUB_4              0x05
#define LSM6DSO_SENSOR_HUB_5              0x06
#define LSM6DSO_SENSOR_HUB_6              0x07
#define LSM6DSO_SENSOR_HUB_7              0x08
#define LSM6DSO_SENSOR_HUB_8              0x09
#define LSM6DSO_SENSOR_HUB_9              0x0A
#define LSM6DSO_SENSOR_HUB_10             0x0B
#define LSM6DSO_SENSOR_HUB_11             0x0C
#define LSM6DSO_SENSOR_HUB_12             0x0D
#define LSM6DSO_SENSOR_HUB_13             0x0E
#define LSM6DSO_SENSOR_HUB_14             0x0F
#define LSM6DSO_SENSOR_HUB_15             0x10
#define LSM6DSO_SENSOR_HUB_16             0x11
#define LSM6DSO_SENSOR_HUB_17             0x12
#define LSM6DSO_SENSOR_HUB_18             0x13
#define LSM6DSO_MASTER_CONFIG             0x14
#define LSM6DSO_SLV0_ADD                  0x15
#define LSM6DSO_SLV0_SUBADD               0x16
#define LSM6DSO_SLV0_CONFIG               0x17
#define LSM6DSO_SLV1_ADD                  0x18
#define LSM6DSO_SLV1_SUBADD               0x19
#define LSM6DSO_SLV1_CONFIG               0x1A
#define LSM6DSO_SLV2_ADD                  0x1B
#define LSM6DSO_SLV2_SUBADD               0x1C
#define LSM6DSO_SLV2_CONFIG               0x1D
#define LSM6DSO_SLV3_ADD                  0x1E
#define LSM6DSO_SLV3_SUBADD               0x1F
#define LSM6DSO_SLV3_CONFIG               0x20
#define LSM6DSO_DATAWRITE_SLV0            0x21
#define LSM6DSO_STATUS_MASTER             0x22



#define LSM6DSO_ADDRESS           0x6B   // Address of LSM6DSO accel/gyro when ADO = 0


#define AFS_2G  0x00
#define AFS_4G  0x02
#define AFS_8G  0x03
#define AFS_16G 0x01

#define GFS_125DPS  0x01
#define GFS_250DPS  0x00
#define GFS_500DPS  0x02
#define GFS_1000DPS 0x04
#define GFS_2000DPS 0x06

#define AODR_12_5Hz  0x01  // same for accel and gyro in normal mode
#define AODR_26Hz    0x02
#define AODR_52Hz    0x03
#define AODR_104Hz   0x04
#define AODR_208Hz   0x05
#define AODR_416Hz   0x06
#define AODR_833Hz   0x07
#define AODR_1660Hz  0x08
#define AODR_3330Hz  0x09
#define AODR_6660Hz  0x0A

#define GODR_12_5Hz  0x01   
#define GODR_26Hz    0x02
#define GODR_52Hz    0x03
#define GODR_104Hz   0x04
#define GODR_208Hz   0x05
#define GODR_416Hz   0x06
#define GODR_833Hz   0x07
#define GODR_1660Hz  0x08
#define GODR_3330Hz  0x09
#define GODR_6660Hz  0x0A


class LSM6DSO
{
  public:
  LSM6DSO(I2Cdev* i2c_bus);
  float getAres(uint8_t Ascale);
  float getGres(uint8_t Gscale);
  uint8_t getChipID();
  void passthruMode();
  void masterMode();
  void sleepMode();
  void wakeMode(uint8_t AODR, uint8_t GODR);
  void init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
  void AGoffsetBias(float * dest1, float * dest2);
  void MagoffsetBias(float * dest1, float * dest2);
  void reset();
  void selfTest();
  void readAccelGyroData(int16_t * destination);
  void readMagData(int16_t * destination);
  int32_t readBaroData();
  int16_t readBaroTemp();
  private:
  float _aRes, _gRes;
  I2Cdev* _i2c_bus;
};

#endif
