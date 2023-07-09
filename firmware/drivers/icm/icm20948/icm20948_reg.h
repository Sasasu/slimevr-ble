#pragma once

#include <assert.h>
#include <stdint.h>

#define ICM20948_ID 0xEA

#define IN(x) (((uint8_t *)&x)[0])
#define OUT(x) (((uint8_t *)(&x)))

#define READ (1 << 7)
#define WRITE (0 << 7)

#define WHO_AM_I_B0 0x00
typedef uint8_t ICM20948_WHO_AM_I_t;

#define USER_CTRL_B0 0x03
typedef struct {
  uint8_t _ : 1;
  uint8_t I2C_MST_RST : 1;
  uint8_t SRAM_RST : 1;
  uint8_t DMP_RST : 1;
  uint8_t I2C_IF_DIS : 1;
  uint8_t I2C_MST_EN : 1;
  uint8_t FIFO_EN : 1;
  uint8_t DMP_EN : 1;
} ICM20948_USER_CTRL_t;

#define LP_CONFIG_B0 0x05
typedef struct {
  uint8_t __ : 4;
  uint8_t GYRO_CYCLE : 1;
  uint8_t ACCEL_CYCLE : 1;
  uint8_t I2C_MST_CYCLE : 1;
  uint8_t _ : 1;
} ICM20948_LP_CONFIG_t;

#define PWR_MGMT_1_B0 0x06
#define ICM20948_MGMT_1_B0_CLKSET_Internal_20Mhz 0x0
#define ICM20948_MGMT_1_B0_CLKSET_Auto 0x1
#define ICM20948_MGMT_1_B0_CLKSET_Stops 0x7
typedef struct {
  uint8_t CLKSEL : 3;
  uint8_t TEMP_DIS : 1;
  uint8_t _ : 1;
  uint8_t LP_EN : 1;
  uint8_t SLEEP : 1;
  uint8_t DEVICE_RESET : 1;
} ICM20948_PWR_MGMT_1_t;

#define PWR_MGMT_2_B0 0x07
typedef struct {
  uint8_t DISABLE_GYRO : 3;
  uint8_t DIABLE_ACCEL : 3;
  uint8_t _ : 2;
} ICM20948_PWR_MGMT_2_t;

#define INT_PIN_CFG_B0 0x0F
typedef struct {
  uint8_t _ : 1;
  uint8_t BYPASS_EN : 1;
  uint8_t FSYNC_INT_MODE_EN : 1;
  uint8_t ACTL_FSYNC : 1;
  uint8_t INT_ANYRD_2CLEAR : 1;
  uint8_t INT1_LATCH_EN : 1;
  uint8_t INT1_OPEN : 1;
  uint8_t INT1_ACTL : 1;
} ICM20948_INT_PIN_CFG_t;

#define INT_ENABLE_B0 0x10
typedef struct {
  uint8_t I2C_MST_INT_EN : 1;
  uint8_t DMP_INT1_EN : 1;
  uint8_t PLL_READY_EN : 1;
  uint8_t WOM_INT_EN : 1;
  uint8_t _ : 3;
  uint8_t REG_WOF_EN : 1;
} ICM20948_INT_ENABLE_t;

#define INT_ENABLE_1_B0 0x11
typedef struct {
  uint8_t RAW_DATA_0_RDY_EN : 1;
  uint8_t _ : 7;
} ICM20948_INT_ENABLE_1_t;

#define INT_ENABLE_2_B0 0x12
typedef struct {
  uint8_t FIFO_OVERFLOW_EN : 5;
  uint8_t _ : 3;
} ICM20948_INT_ENABLE_2_t;

#define INT_ENABLE_3_B0 0x13
typedef struct {
  uint8_t FIFO_WM_EN : 5;
  uint8_t _ : 3;
} ICM20948_INT_ENABLE_3_t;

#define I2C_MST_STATUS_B0 0x17
typedef struct {
  uint8_t I2C_PERIPH0_NACK : 1;
  uint8_t I2C_PERIPH1_NACK : 1;
  uint8_t I2C_PERIPH2_NACK : 1;
  uint8_t I2C_PERIPH3_NACK : 1;
  uint8_t I2C_PERIPH4_NACK : 1;
  uint8_t I2C_LOST_ARB : 1;
  uint8_t I2C_PERIPH4_DONE : 1;
  uint8_t PASS_THROUGH : 1;
} ICM20948_I2C_MST_STATUS_t;

#define INT_STATUS_B0 0x19
typedef struct {
  uint8_t I2C_MST_INT : 1;
  uint8_t DMP_INT1 : 1;
  uint8_t PLL_RDY_INT : 1;
  uint8_t WOM_INT : 1;
  uint8_t _ : 4;
} ICM20948_INT_STATUS_t;

#define INT_STATUS_1_B0 0x1A
typedef struct {
  uint8_t RAW_DATA_0_RDY_INT : 1;
  uint8_t _ : 7;
} ICM_20948_INT_STATUS_1_t;

#define INT_STATUS_2_B0 0x1B
typedef struct {
  uint8_t FIFO_OVERFLOW_INT : 5;
  uint8_t _ : 3;
} ICM_20948_INT_STATUS_2_t;

#define INT_STATUS_3_B0 0x1C
typedef struct {
  uint8_t FIFO_WM_EN : 5;
  uint8_t _ : 3;
} ICM_20948_INT_STATUS_3_t;

#define DELAY_TIMEH_B0 0x28
typedef uint8_t ICM20948_DELAY_TIMEH_t;

#define DELAY_TIMEL_B0 0x29
typedef uint8_t ICM20948_DELAY_TIMEL_t;

#define ACCEL_XOUT_H_B0 0x2D
typedef uint8_t ICM20948_ACCEL_XOUT_H_t;

#define ACCEL_XOUT_L_B0 0x2E
typedef uint8_t ICM20948_ACCEL_XOUT_L_t;

#define ACCEL_YOUT_H_B0 0x2F
typedef uint8_t ICM20948_ACCEL_YOUT_H_t;

#define ACCEL_YOUT_L_B0 0x30
typedef uint8_t ICM20948_ACCEL_YOUT_L_t;

#define ACCEL_ZOUT_H_B0 0x31
typedef uint8_t ICM20948_ACCEL_ZOUT_H_t;

#define ACCEL_ZOUT_L_B0 0x32
typedef uint8_t ICM20948_ACCEL_ZOUT_L_t;

#define GYRO_XOUT_H_B0 0x33
typedef uint8_t ICM20948_GYRO_XOUT_H_t;

#define GYRO_XOUT_L_B0 0x34
typedef uint8_t ICM20948_GYRO_XOUT_L_t;

#define GYRO_YOUT_H_B0 0x35
typedef uint8_t ICM20948_GYRO_YOUT_H_t;

#define GYRO_YOUT_L_B0 0x36
typedef uint8_t ICM20948_GYRO_YOUT_L_t;

#define GYRO_ZOUT_H_B0 0x37
typedef uint8_t ICM20948_GYRO_ZOUT_H_t;

#define GYRO_ZOUT_L_B0 0x38
typedef uint8_t ICM20948_GYRO_ZOUT_L_t;

#define TEMP_OUT_H_B0 0x39
typedef uint8_t ICM20948_TEMP_OUT_H_t;

#define TEMP_OUT_L_B0 0x3A
typedef uint8_t ICM20948_TEMP_OUT_L_t;

#define EXT_SLV_SENS_DATA_00_B0 0x3B
#define EXT_SLV_SENS_DATA_01_B0 0x3C
#define EXT_SLV_SENS_DATA_02_B0 0x3D
#define EXT_SLV_SENS_DATA_03_B0 0x3E
#define EXT_SLV_SENS_DATA_04_B0 0x3F
#define EXT_SLV_SENS_DATA_05_B0 0x40
#define EXT_SLV_SENS_DATA_06_B0 0x41
#define EXT_SLV_SENS_DATA_07_B0 0x42
#define EXT_SLV_SENS_DATA_08_B0 0x43
#define EXT_SLV_SENS_DATA_09_B0 0x44
#define EXT_SLV_SENS_DATA_10_B0 0x45
#define EXT_SLV_SENS_DATA_11_B0 0x46
#define EXT_SLV_SENS_DATA_12_B0 0x47
#define EXT_SLV_SENS_DATA_13_B0 0x48
#define EXT_SLV_SENS_DATA_14_B0 0x49
#define EXT_SLV_SENS_DATA_15_B0 0x4A
#define EXT_SLV_SENS_DATA_16_B0 0x4B
#define EXT_SLV_SENS_DATA_17_B0 0x4C
#define EXT_SLV_SENS_DATA_18_B0 0x4D
#define EXT_SLV_SENS_DATA_19_B0 0x4E
#define EXT_SLV_SENS_DATA_20_B0 0x4F
#define EXT_SLV_SENS_DATA_21_B0 0x50
#define EXT_SLV_SENS_DATA_22_B0 0x51
#define EXT_SLV_SENS_DATA_23_B0 0x52
typedef uint8_t ICM20948_EXT_SLV_SENS_DATA_t;

#define FIFO_EN_1_B0 0x66
typedef struct {
  uint8_t SLV_0_FIFO_EN : 1;
  uint8_t SLV_1_FIFO_EN : 1;
  uint8_t SLV_2_FIFO_EN : 1;
  uint8_t SLV_3_FIFO_EN : 1;
  uint8_t _ : 4;
} ICM20948_FIFO_EN_1_t;

#define FIFO_EN_2_B0 0x67
typedef struct {
  uint8_t TEMP_FIFO_EN : 1;
  uint8_t GYRO_X_FIFO_EN : 1;
  uint8_t GYRO_Y_FIFO_EN : 1;
  uint8_t GYRO_Z_FIFO_EN : 1;
  uint8_t ACCEL_FIFO_EN : 1;
  uint8_t _ : 3;
} ICM_20948_FIFO_EN_2_t;

#define FIFO_RST_B0 0x68
typedef struct {
  uint8_t FIFO_RESET : 5;
  uint8_t _ : 3;
} ICM20948_FIFO_RST_t;

#define FIFO_MODE_B0 0x69
typedef struct {
  uint8_t FIFO_MODE : 5;
  uint8_t _ : 3;
} ICM20948_FIFO_MODE_t;

#define FIFO_COUNTH_B0 0x70
typedef struct {
  uint8_t _ : 3;
  uint8_t FIFO_CNT : 5;
} ICM20948_FIFO_COUNT_H_t;

#define FIFO_COUNTL_B0 0x71
typedef uint8_t ICM20948_FIFO_COUNT_L_t;

#define FIFO_R_W_B0 0x72
typedef uint8_t ICM20948_FIFO_R_W_t;

#define DATA_RDY_STATUS_B0 0x74
typedef struct {
  uint8_t RAW_DATA_RDY : 4;
  uint8_t _ : 3;
  uint8_t WOF_STATUS : 1;
} ICM20948_DATA_RDY_STATUS_t;

#define FIFO_CFG_B0 0x76
typedef struct {
  uint8_t FIFO_CFG : 1;
  uint8_t _ : 7;
} ICM20948_FIFO_CFG_t;

#define REG_BANK_SEL_B0 0x7F
typedef struct {
  uint8_t __ : 4;
  uint8_t USER_BANK : 2;
  uint8_t _ : 2;
} ICM_20948_REG_BANK_SEL_t;

#define SELF_TEST_X_GYRO_B1 0x02
typedef uint8_t ICM20948_SELF_TEST_X_GYRO_t;

#define SELF_TEST_Y_GYRO_B1 0x03
typedef uint8_t ICM20948_SELF_TEST_Y_GYRO_t;

#define SELF_TEST_Z_GYRO_B1 0x04
typedef uint8_t ICM20948_SELF_TEST_Z_GYRO_t;

#define SELF_TEST_X_ACCEL_B1 0x0E
typedef uint8_t ICM20948_SELF_TEST_X_ACCEL_t;

#define SELF_TEST_Y_ACCEL_B1 0x0F
typedef uint8_t ICM20948_SELF_TEST_Y_ACCEL_t;

#define SELF_TEST_Z_ACCEL_B1 0x10
typedef uint8_t ICM20948_SELF_TEST_Z_ACCEL_t;

#define XA_OFFS_H_B1 0x14
typedef uint8_t ICM20948_XA_OFFS_H_t;

#define XA_OFFS_L_B1 0x15
typedef struct {
  uint8_t _ : 1;
  uint8_t XA_OFFS : 7;
} ICM20948_XA_OFFS_L_t;

#define YA_OFFS_H_B1 0x17
typedef uint8_t ICM20948_YA_OFFS_H_t;

#define YA_OFFS_L_B1 0x18
typedef struct {
  uint8_t _ : 1;
  uint8_t YA_OFFS : 7;
} ICM20948_YA_OFFS_L_t;

#define ZA_OFFS_H_B1 0x1A
typedef uint8_t ICM20948_ZA_OFFS_H_t;

#define ZA_OFFS_L_B1 0x1B
typedef struct {
  uint8_t _ : 1;
  uint8_t ZA_OFFS : 7;
} ICM20948_ZA_OFFS_L_t;

#define TIMEBASE_CORRECTION_PLL_B1 0x28
typedef uint8_t ICM20948_TIMEBASE_CORRECTION_PLL_t;

#define REG_BANK_SEL_B1 0x7F

#define GYRO_SMPLRT_DIV_B2 0x00
typedef uint8_t ICM20948_GYRO_SMPLRT_DIV_t;

#define GYRO_CONFIG_1_B2 0x01
typedef struct {
  uint8_t GYRO_FCHOICE : 1;
  uint8_t GYRO_FS_SEL : 2;
  uint8_t GYRO_DLPFCFG : 3;
  uint8_t _ : 2;
} ICM20948_GYRO_CONFIG_1_t;

#define GYRO_CONFIG_2_B2 0x02
typedef struct {
  uint8_t GYRO_AVGCFG : 3;
  uint8_t ZGYRO_CTEN : 1;
  uint8_t YGYRO_CTEN : 1;
  uint8_t XGYRO_CTEN : 1;
  uint8_t _ : 2;
} ICM20948_GYRO_CONFIG_2_t;

#define XG_OFFS_USRH_B2 0x03
typedef uint8_t ICM20948_XG_OFFS_USRH_t;

#define XG_OFFS_USRL_B2 0x04
typedef uint8_t ICM20948_XG_OFFS_USRL_t;

#define YG_OFFS_USRH_B2 0x05
typedef uint8_t ICM20948_YG_OFFS_USRH_t;

#define YG_OFFS_USRL_B2 0x06
typedef uint8_t ICM20948_YG_OFFS_USRL_t;

#define ZG_OFFS_USRH_B2 0x07
typedef uint8_t ICM20948_ZG_OFFS_USRH_t;

#define ZG_OFFS_USRL_B2 0x08
typedef uint8_t ICM20948_ZG_OFFS_USRL_t;

#define ODR_ALIGN_EN_B2 0x09
typedef struct {
  uint8_t ODR_ALIGN_EN : 1;
  uint8_t _ : 7;
} ICM20948_ODR_ALIGN_EN_t;

#define ACCEL_SMPLRT_DIV_1_B2 0x10
typedef struct {
  uint8_t ACCEL_SMPLRT_DIV : 4;
  uint8_t _ : 4;
} ICM20948_ACCEL_SMPLRT_DIV_1_t;

#define ACCEL_SMPLRT_DIV_2_B2 0x11
typedef uint8_t ICM20948_ACCEL_SMPLRT_DIV_2_t;

#define ACCEL_INTEL_CTRL_B2 0x12
typedef struct {
  uint8_t ACCEL_INTEL_MODE_INT : 1;
  uint8_t ACCEL_INTEL_EN : 1;
  uint8_t _ : 6;
} ICM20948_ACCEL_INTEL_CTRL_t;

#define ACCEL_WOM_THR_B2 0x13
typedef uint8_t ICM20948_ACCEL_WOM_THR_t;

#define ACCEL_CONFIG_B2 0x14
typedef struct {
  uint8_t ACCEL_FCHOICE : 1;
  uint8_t ACCEL_FS_SEL : 2;
  uint8_t ACCEL_DLPFCFG : 3;
  uint8_t _ : 2;
} ICM20948_ACCEL_CONFIG_t;

#define ACCEL_CONFIG_2_B2 0x15
typedef struct {
  uint8_t DEC3_CFG : 2;
  uint8_t AZ_ST_EN : 1;
  uint8_t AY_ST_EN : 1;
  uint8_t AX_ST_EN : 1;
  uint8_t _ : 3;
} ICM20948_ACCEL_CONFIG_2_t;

#define FSYNC_CONFIG_B2 0x52
typedef struct {
  uint8_t EXT_SYNC_SET : 4;
  uint8_t WOF_EDGE_INT : 1;
  uint8_t WOF_DEGLITCH_EN : 1;
  uint8_t _ : 1;
  uint8_t DELAY_TIME_EN : 1;
} ICM20948_FSYNC_CONFIG_t;

#define TEMP_CONFIG_B2 0x53
typedef struct {
  uint8_t TEMP_DLPFCFG : 3;
  uint8_t _ : 5;
} ICM20948_TEMP_CONFIG_t;

#define MOD_CTRL_USR_B2 0x14
typedef struct {
  uint8_t REG_LP_DMP_EN : 1;
  uint8_t _ : 7;
} ICM20948_MOD_CTRL_USR_t;

#define REG_BANK_SEL_B2 0x7F

#define I2C_MST_ODR_CONFIG_B3 0x00
typedef struct {
  uint8_t I2C_MST_ODR_CONFIG : 4;
  uint8_t _ : 4;
} ICM20948_I2C_MST_ODR_CONFIG_t;

#define I2C_MST_CTRL_B3 0x01
typedef struct {
  uint8_t I2C_MST_CLK : 4;
  uint8_t I2C_MST_P_NSR : 1;
  uint8_t _ : 2;
  uint8_t MULT_MST_EN : 1;
} ICM20948_I2C_MST_CTRL_t;

#define I2C_MST_DELAY_CTRL_B3 0x02
typedef struct {
  uint8_t I2C_SLV0_DELAY_EN : 1;
  uint8_t I2C_SLV1_DELAY_EN : 1;
  uint8_t I2C_SLV2_DELAY_EN : 1;
  uint8_t I2C_SLV3_DELAY_EN : 1;
  uint8_t I2C_SLV4_DELAY_EN : 1;
  uint8_t _ : 2;
  uint8_t DELAY_ES_SHADOW : 1;
} ICM20948_I2C_MST_DELAY_CTRL_t;

#define I2C_SLV0_ADDR_B3 0x03
#define I2C_SLV1_ADDR_B3 0x07
#define I2C_SLV2_ADDR_B3 0x0B
#define I2C_SLV3_ADDR_B3 0x0F
#define I2C_SLV4_ADDR_B3 0x13
typedef struct {
  uint8_t ID : 7;
  uint8_t RNW : 1;
} ICM20948_I2C_SLV_ADDR_t;

#define I2C_SLV0_REG_B3 0x04
#define I2C_SLV1_REG_B3 0x08
#define I2C_SLV2_REG_B3 0x0C
#define I2C_SLV3_REG_B3 0x10
#define I2C_SLV4_REG_B3 0x14
typedef uint8_t ICM20948_I2C_SLV_REG_t;

#define I2C_SLV0_CTRL_B3 0x05
#define I2C_SLV1_CTRL_B3 0x09
#define I2C_SLV2_CTRL_B3 0x0D
#define I2C_SLV3_CTRL_B3 0x11
#define I2C_SLV4_CTRL_B3 0x15
typedef struct {
  uint8_t LENG : 4;
  uint8_t GRP : 1;
  uint8_t REG_DIS : 1;
  uint8_t BYTE_SW : 1;
  uint8_t EN : 1;
} ICM20948_I2C_SLV_CTRL_t;

#define I2C_SLV0_DO_B3 0x06
#define I2C_SLV1_DO_B3 0x0A
#define I2C_SLV2_DO_B3 0x0E
#define I2C_SLV3_DO_B3 0x12
#define I2C_SLV4_DO_B3 0x16
typedef uint8_t ICM20948_I2C_SLV_DO_t;

#define I2C_SLV4_DI_B3 0x17
typedef uint8_t ICM20948_I2C_SLV_DI_t;

#define REG_BANK_SEL_B3 0x7F
