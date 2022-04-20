/*
 * MPU6050.h
 *
 *           Author: steveb
 *       Created on: March 04, 2022
 */

#ifndef MPU6050_H_
#define MPU6050_H_

typedef uint8_t MPU6050_REGISTER;
typedef uint8_t AK8963_REGISTER;

#define MPU6050_ADDRESS                       0x68

#define MPU6050_REGISTER_SELF_TEST_X          ((MPU6050_REGISTER)0x0D)
#define MPU6050_REGISTER_SELF_TEST_Y          ((MPU6050_REGISTER)0x0E)
#define MPU6050_REGISTER_SELF_TEST_Z          ((MPU6050_REGISTER)0x0F)
#define MPU6050_REGISTER_SELF_TEST_A          ((MPU6050_REGISTER)0x10)
#define MPU6050_REGISTER_SMPLRT_DIV           ((MPU6050_REGISTER)0x19)
#define MPU6050_REGISTER_CONFIG               ((MPU6050_REGISTER)0x1A)
#define MPU6050_REGISTER_GYRO_CONFIG          ((MPU6050_REGISTER)0x1B)
#define MPU6050_REGISTER_ACCEL_CONFIG         ((MPU6050_REGISTER)0x1C)
#define MPU6050_REGISTER_FIFO_EN              ((MPU6050_REGISTER)0x23)
#define MPU6050_REGISTER_I2C_MST_CTRL         ((MPU6050_REGISTER)0x24)
#define MPU6050_REGISTER_I2C_SLV0_ADDR        ((MPU6050_REGISTER)0x25)
#define MPU6050_REGISTER_I2C_SLV0_REG         ((MPU6050_REGISTER)0x26)
#define MPU6050_REGISTER_I2C_SLV0_CTRL        ((MPU6050_REGISTER)0x27)
#define MPU6050_REGISTER_I2C_SLV1_ADDR        ((MPU6050_REGISTER)0x28)
#define MPU6050_REGISTER_I2C_SLV1_REG         ((MPU6050_REGISTER)0x29)
#define MPU6050_REGISTER_I2C_SLV1_CTRL        ((MPU6050_REGISTER)0x2A)
#define MPU6050_REGISTER_I2C_SLV2_ADDR        ((MPU6050_REGISTER)0x2B)
#define MPU6050_REGISTER_I2C_SLV2_REG         ((MPU6050_REGISTER)0x2C)
#define MPU6050_REGISTER_I2C_SLV2_CTRL        ((MPU6050_REGISTER)0x2D)
#define MPU6050_REGISTER_I2C_SLV3_ADDR        ((MPU6050_REGISTER)0x2E)
#define MPU6050_REGISTER_I2C_SLV3_REG         ((MPU6050_REGISTER)0x2F)
#define MPU6050_REGISTER_I2C_SLV3_CTRL        ((MPU6050_REGISTER)0x30)
#define MPU6050_REGISTER_I2C_SLV4_ADDR        ((MPU6050_REGISTER)0x31)
#define MPU6050_REGISTER_I2C_SLV4_REG         ((MPU6050_REGISTER)0x32)
#define MPU6050_REGISTER_I2C_SLV4_DO          ((MPU6050_REGISTER)0x33)
#define MPU6050_REGISTER_I2C_SLV4_CTRL        ((MPU6050_REGISTER)0x34)
#define MPU6050_REGISTER_I2C_SLV4_DI          ((MPU6050_REGISTER)0x35)
#define MPU6050_REGISTER_I2C_MST_STATUS       ((MPU6050_REGISTER)0x36)
#define MPU6050_REGISTER_INT_PIN_CFG          ((MPU6050_REGISTER)0x37)
#define MPU6050_REGISTER_INT_ENABLE           ((MPU6050_REGISTER)0x38)
#define MPU6050_REGISTER_INT_STATUS           ((MPU6050_REGISTER)0x3A)
#define MPU6050_REGISTER_ACCEL_XOUT_H         ((MPU6050_REGISTER)0x3B)
#define MPU6050_REGISTER_ACCEL_XOUT_L         ((MPU6050_REGISTER)0x3C)
#define MPU6050_REGISTER_ACCEL_YOUT_H         ((MPU6050_REGISTER)0x3D)
#define MPU6050_REGISTER_ACCEL_YOUT_L         ((MPU6050_REGISTER)0x3E)
#define MPU6050_REGISTER_ACCEL_ZOUT_H         ((MPU6050_REGISTER)0x3F)
#define MPU6050_REGISTER_ACCEL_ZOUT_L         ((MPU6050_REGISTER)0x40)
#define MPU6050_REGISTER_TEMP_OUT_H           ((MPU6050_REGISTER)0x41)
#define MPU6050_REGISTER_TEMP_OUT_L           ((MPU6050_REGISTER)0x42)
#define MPU6050_REGISTER_GYRO_XOUT_H          ((MPU6050_REGISTER)0x43)
#define MPU6050_REGISTER_GYRO_XOUT_L          ((MPU6050_REGISTER)0x44)
#define MPU6050_REGISTER_GYRO_YOUT_H          ((MPU6050_REGISTER)0x45)
#define MPU6050_REGISTER_GYRO_YOUT_L          ((MPU6050_REGISTER)0x46)
#define MPU6050_REGISTER_GYRO_ZOUT_H          ((MPU6050_REGISTER)0x47)
#define MPU6050_REGISTER_GYRO_ZOUT_L          ((MPU6050_REGISTER)0x48)
#define MPU6050_REGISTER_EXT_SENS_DATA_00     ((MPU6050_REGISTER)0x49)
#define MPU6050_REGISTER_EXT_SENS_DATA_01     ((MPU6050_REGISTER)0x4A)
#define MPU6050_REGISTER_EXT_SENS_DATA_02     ((MPU6050_REGISTER)0x4B)
#define MPU6050_REGISTER_EXT_SENS_DATA_03     ((MPU6050_REGISTER)0x4C)
#define MPU6050_REGISTER_EXT_SENS_DATA_04     ((MPU6050_REGISTER)0x4D)
#define MPU6050_REGISTER_EXT_SENS_DATA_05     ((MPU6050_REGISTER)0x4E)
#define MPU6050_REGISTER_EXT_SENS_DATA_06     ((MPU6050_REGISTER)0x4F)
#define MPU6050_REGISTER_EXT_SENS_DATA_07     ((MPU6050_REGISTER)0x50)
#define MPU6050_REGISTER_EXT_SENS_DATA_08     ((MPU6050_REGISTER)0x51)
#define MPU6050_REGISTER_EXT_SENS_DATA_09     ((MPU6050_REGISTER)0x52)
#define MPU6050_REGISTER_EXT_SENS_DATA_10     ((MPU6050_REGISTER)0x53)
#define MPU6050_REGISTER_EXT_SENS_DATA_11     ((MPU6050_REGISTER)0x54)
#define MPU6050_REGISTER_EXT_SENS_DATA_12     ((MPU6050_REGISTER)0x55)
#define MPU6050_REGISTER_EXT_SENS_DATA_13     ((MPU6050_REGISTER)0x56)
#define MPU6050_REGISTER_EXT_SENS_DATA_14     ((MPU6050_REGISTER)0x57)
#define MPU6050_REGISTER_EXT_SENS_DATA_15     ((MPU6050_REGISTER)0x58)
#define MPU6050_REGISTER_EXT_SENS_DATA_16     ((MPU6050_REGISTER)0x59)
#define MPU6050_REGISTER_EXT_SENS_DATA_17     ((MPU6050_REGISTER)0x5A)
#define MPU6050_REGISTER_EXT_SENS_DATA_18     ((MPU6050_REGISTER)0x5B)
#define MPU6050_REGISTER_EXT_SENS_DATA_19     ((MPU6050_REGISTER)0x5C)
#define MPU6050_REGISTER_EXT_SENS_DATA_20     ((MPU6050_REGISTER)0x5D)
#define MPU6050_REGISTER_EXT_SENS_DATA_21     ((MPU6050_REGISTER)0x5E)
#define MPU6050_REGISTER_EXT_SENS_DATA_22     ((MPU6050_REGISTER)0x5F)
#define MPU6050_REGISTER_EXT_SENS_DATA_23     ((MPU6050_REGISTER)0x60)
#define MPU6050_REGISTER_I2C_SLV0_DO          ((MPU6050_REGISTER)0x63)
#define MPU6050_REGISTER_I2C_SLV1_DO          ((MPU6050_REGISTER)0x64)
#define MPU6050_REGISTER_I2C_SLV2_DO          ((MPU6050_REGISTER)0x65)
#define MPU6050_REGISTER_I2C_SLV3_DO          ((MPU6050_REGISTER)0x66)
#define MPU6050_REGISTER_I2C_MST_DELAY_CTRL   ((MPU6050_REGISTER)0x67)
#define MPU6050_REGISTER_SIGNAL_PATH_RESET    ((MPU6050_REGISTER)0x68)
#define MPU6050_REGISTER_USER_CTRL            ((MPU6050_REGISTER)0x6A)
#define MPU6050_REGISTER_PWR_MGMT_1           ((MPU6050_REGISTER)0x6B)
#define MPU6050_REGISTER_PWR_MGMT_2           ((MPU6050_REGISTER)0x6C)
#define MPU6050_REGISTER_FIFO_COUNTH          ((MPU6050_REGISTER)0x72)
#define MPU6050_REGISTER_FIFO_COUNTL          ((MPU6050_REGISTER)0x73)
#define MPU6050_REGISTER_FIFO_R_W             ((MPU6050_REGISTER)0x74)
#define MPU6050_REGISTER_WHO_AM_I             ((MPU6050_REGISTER)0x75)


union MPU6050_transaction
{
  struct MPU6050_transaction_field
  {
    struct MPU6050_transaction_field_command
    {
      MPU6050_REGISTER  AD:7;
      uint8_t           R_W:1; /* 0=write, 1=read */
    }       address;
    union MPU6050_transaction_field_data
    {
      struct MPU6050_SELF_TEST_X
      {
        uint8_t         XG_TEST:5;
        uint8_t         XA_TEST:3;
      }               SELF_TEST_X;
      struct MPU6050_SELF_TEST_Y
      {
        uint8_t         YG_TEST:5;
        uint8_t         YA_TEST:3;
      }               SELF_TEST_Y;
      struct MPU6050_SELF_TEST_Z
      {
        uint8_t         ZG_TEST:5;
        uint8_t         ZA_TEST:3;
      }               SELF_TEST_Z;
      struct MPU6050_SELF_TEST_A
      {
        uint8_t       ZA_TEST:2;
        uint8_t       YA_TEST:2;
        uint8_t       XA_TEST:2;
        uint8_t       RESERVED:2;
      }               SELF_TEST_A;
      struct MPU6050_SMPLRT_DIV
      {
        uint8_t         SMPLRT_DIV:8;
      }               SMPLRT_DIV;
      struct MPU6050_CONFIG
      {
        uint8_t         DLPF_CFG:3;
        uint8_t         EXT_SYNC_SET:3;
        uint8_t         reserved:2;
      }               CONFIG;
      struct MPU6050_GYRO_CONFIG
      {
        uint8_t         reserved0:3;
        uint8_t         FS_SEL:2;
        uint8_t         reserved1:3;
      }               GYRO_CONFIG;
      struct MPU6050_ACCEL_CONFIG
      {
        uint8_t         reserved:3;
        uint8_t         ACCEL_FS_SEL:2;
        uint8_t         ZA_ST:1;
        uint8_t         YA_ST:1;
        uint8_t         XA_ST:1;
      }               ACCEL_CONFIG;
      struct MPU6050_FIFO_EN
      {
        uint8_t         SLV0_FIFO_EN:1;
        uint8_t         SLV1_FIFO_EN:1;
        uint8_t         SLV2_FIFO_EN:1;
        uint8_t         ACCEL_FIFO_EN:1;
        uint8_t         ZG_FIFO_EN:1;
        uint8_t         YG_FIFO_EN:1;
        uint8_t         XG_FIFO_EN:1;
        uint8_t         TEMP_FIFO_EN:1;
      }               FIFO_EN;

      struct MPU6050_I2C_MST_CTRL
      {
        uint8_t         I2C_MST_CLK:4;
        uint8_t         I2C_MST_P_NSR:1;
        uint8_t         SLV_3_FIFO_EN:1;
        uint8_t         WAIT_FOR_ES:1;
        uint8_t         MULT_MST_EN:1;
      }               I2C_MST_CTRL;
      struct MPU6050_I2C_SLV0_ADDR
      {
        uint8_t         I2C_ID_0:7;
        uint8_t         I2C_SLV0_RW:1;
      }               I2C_SLV0_ADDR;
      struct MPU6050_I2C_SLV0_REG
      {
        uint8_t         I2C_SLV0_REG:8;
      }               I2C_SLV0_REG;
      struct MPU6050_I2C_SLV0_CTRL
      {
        uint8_t         I2C_SLV0_LENG:4;
        uint8_t         I2C_SLV0_GRP:1;
        uint8_t         I2C_SLV0_REG_DIS:1;
        uint8_t         I2C_SLV0_BYTE_SW:1;
        uint8_t         I2C_SLV0_EN:1;
      }               I2C_SLV0_CTRL;
      struct MPU6050_I2C_SLV1_ADDR
      {
        uint8_t         I2C_ID_1:7;
        uint8_t         I2C_SLV1_RW:1;
      }               I2C_SLV1_ADDR;
      struct MPU6050_I2C_SLV1_REG
      {
        uint8_t         I2C_SLV1_REG:8;
      }               I2C_SLV1_REG;
      struct MPU6050_I2C_SLV1_CTRL
      {
        uint8_t         I2C_SLV1_LENG:4;
        uint8_t         I2C_SLV1_GRP:1;
        uint8_t         I2C_SLV1_REG_DIS:1;
        uint8_t         I2C_SLV1_BYTE_SW:1;
        uint8_t         I2C_SLV1_EN:1;
      }               I2C_SLV1_CTRL;
      struct MPU6050_I2C_SLV2_ADDR
      {
        uint8_t         I2C_ID_2:7;
        uint8_t         I2C_SLV2_RW:1;
      }               I2C_SLV2_ADDR;
      struct MPU6050_I2C_SLV2_REG
      {
        uint8_t         I2C_SLV2_REG:8;
      }               I2C_SLV2_REG;
      struct MPU6050_I2C_SLV2_CTRL
      {
        uint8_t         I2C_SLV2_LENG:4;
        uint8_t         I2C_SLV2_GRP:1;
        uint8_t         I2C_SLV2_REG_DIS:1;
        uint8_t         I2C_SLV2_BYTE_SW:1;
        uint8_t         I2C_SLV2_EN:1;
      }               I2C_SLV2_CTRL;
      struct MPU6050_I2C_SLV3_ADDR
      {
        uint8_t         I2C_ID_3:7;
        uint8_t         I2C_SLV3_RW:1;
      }               I2C_SLV3_ADDR;
      struct MPU6050_I2C_SLV3_REG
      {
        uint8_t         I2C_SLV3_REG:8;
      }               I2C_SLV3_REG;
      struct MPU6050_I2C_SLV3_CTRL
      {
        uint8_t         I2C_SLV3_LENG:4;
        uint8_t         I2C_SLV3_GRP:1;
        uint8_t         I2C_SLV3_REG_DIS:1;
        uint8_t         I2C_SLV3_BYTE_SW:1;
        uint8_t         I2C_SLV3_EN:1;
      }               I2C_SLV3_CTRL;
      struct MPU6050_I2C_SLV4_ADDR
      {
        uint8_t         I2C_ID_4:7;
        uint8_t         I2C_SLV4_RW:1;
      }               I2C_SLV4_ADDR;
      struct MPU6050_I2C_SLV4_REG
      {
        uint8_t         I2C_SLV4_REG:8;
      }               I2C_SLV4_REG;
      struct MPU6050_I2C_SLV4_DO
      {
        uint8_t         I2C_SLV4_DO:8;
      }               I2C_SLV4_DO;
      struct MPU6050_I2C_SLV4_CTRL
      {
        uint8_t         I2C_MST_DLY:5;
        uint8_t         I2C_SLV4_REG_DIS:1;
        uint8_t         I2C_SLV4_INT_EN:1;
        uint8_t         I2C_SLV4_EN:1;
      }               I2C_SLV4_CTRL;
      struct MPU6050_I2C_SLV4_DI
      {
        uint8_t         I2C_SLV4_DI:8;
      }               I2C_SLV4_DI;
      struct MPU6050_I2C_MST_STATUS
      {
        uint8_t         I2C_SLV0_NACK:1;
        uint8_t         I2C_SLV1_NACK:1;
        uint8_t         I2C_SLV2_NACK:1;
        uint8_t         I2C_SLV3_NACK:1;
        uint8_t         I2C_SLV4_NACK:1;
        uint8_t         I2C_LOST_ARB:1;
        uint8_t         I2C_SLV4_DONE:1;
        uint8_t         PASS_THROUGH:1;
      }               I2C_MST_STATUS;
      struct MPU6050_INT_PIN_CFG
      {
        uint8_t         reserved:1;
        uint8_t         I2C_BYPASS_EN:1;
        uint8_t         FSYNC_INT_EN:1;
        uint8_t         FSYNC_INT_LEVEL:1;
        uint8_t         INT_RD_CLEAR:1;
        uint8_t         LATCH_INT_EN:1;
        uint8_t         INT_OPEN:1;
        uint8_t         INT_LEVEL:1;
      }               INT_PIN_CFG;
      struct MPU6050_INT_ENABLE
      {
        uint8_t         DATA_RDY_EN:1;
        uint8_t         reserved0:2;
        uint8_t         I2C_MST_INT_EN:1;
        uint8_t         FIFO_OFLOW_EN:1;
        uint8_t         reserved1:3;
      }               INT_ENABLE;
      struct MPU6050_INT_STATUS
      {
        uint8_t         DATA_RDY_INT:1;
        uint8_t         reserved0:2;
        uint8_t         I2C_MST_INT:1;
        uint8_t         FIFO_OFLOW_INT:1;
        uint8_t         reserved1:3;
      }               INT_STATUS;
      struct MPU6050_ACCEL_XOUT_H
      {
        uint8_t         ACCEL_XOUT:8;
      }               ACCEL_XOUT_H;
      struct MPU6050_ACCEL_XOUT_L
      {
        uint8_t         ACCEL_XOUT:8;
      }               ACCEL_XOUT_L;
      struct MPU6050_ACCEL_YOUT_H
      {
        uint8_t         ACCEL_YOUT:8;
      }               ACCEL_YOUT_H;
      struct MPU6050_ACCEL_YOUT_L
      {
        uint8_t         ACCEL_YOUT:8;
      }               ACCEL_YOUT_L;
      struct MPU6050_ACCEL_ZOUT_H
      {
        uint8_t         ACCEL_ZOUT:8;
      }               ACCEL_ZOUT_H;
      struct MPU6050_ACCEL_ZOUT_L
      {
        uint8_t         ACCEL_ZOUT:8;
      }               ACCEL_ZOUT_L;
      struct MPU6050_TEMP_OUT_H
      {
        uint8_t         TEMP_OUT:8;
      }               TEMP_OUT_H;
      struct MPU6050_TEMP_OUT_L
      {
        uint8_t         TEMP_OUT:8;
      }               TEMP_OUT_L;
      struct MPU6050_GYRO_XOUT_H
      {
        uint8_t         GYRO_XOUT:8;
      }               GYRO_XOUT_H;
      struct MPU6050_GYRO_XOUT_L
      {
        uint8_t         GYRO_XOUT:8;
      }               GYRO_XOUT_L;
      struct MPU6050_GYRO_YOUT_H
      {
        uint8_t         GYRO_YOUT:8;
      }               GYRO_YOUT_H;
      struct MPU6050_GYRO_YOUT_L
      {
        uint8_t         GYRO_YOUT:8;
      }               GYRO_YOUT_L;
      struct MPU6050_GYRO_ZOUT_H
      {
        uint8_t         GYRO_ZOUT:8;
      }               GYRO_ZOUT_H;
      struct MPU6050_GYRO_ZOUT_L
      {
        uint8_t         GYRO_ZOUT:8;
      }               GYRO_ZOUT_L;
      struct MPU6050_EXT_SENS_DATA_00
      {
        uint8_t         EXT_SENS_DATA_00:8;
      }               EXT_SENS_DATA_00;
      struct MPU6050_EXT_SENS_DATA_01
      {
        uint8_t         EXT_SENS_DATA_01:8;
      }               EXT_SENS_DATA_01;
      struct MPU6050_EXT_SENS_DATA_02
      {
        uint8_t         EXT_SENS_DATA_02:8;
      }               EXT_SENS_DATA_02;
      struct MPU6050_EXT_SENS_DATA_03
      {
        uint8_t         EXT_SENS_DATA_03:8;
      }               EXT_SENS_DATA_03;
      struct MPU6050_EXT_SENS_DATA_04
      {
        uint8_t         EXT_SENS_DATA_04:8;
      }               EXT_SENS_DATA_04;
      struct MPU6050_EXT_SENS_DATA_05
      {
        uint8_t         EXT_SENS_DATA_05:8;
      }               EXT_SENS_DATA_05;
      struct MPU6050_EXT_SENS_DATA_06
      {
        uint8_t         EXT_SENS_DATA_06:8;
      }               EXT_SENS_DATA_06;
      struct MPU6050_EXT_SENS_DATA_07
      {
        uint8_t         EXT_SENS_DATA_07:8;
      }               EXT_SENS_DATA_07;
      struct MPU6050_EXT_SENS_DATA_08
      {
        uint8_t         EXT_SENS_DATA_08:8;
      }               EXT_SENS_DATA_08;
      struct MPU6050_EXT_SENS_DATA_09
      {
        uint8_t         EXT_SENS_DATA_09:8;
      }               EXT_SENS_DATA_09;
      struct MPU6050_EXT_SENS_DATA_10
      {
        uint8_t         EXT_SENS_DATA_10:8;
      }               EXT_SENS_DATA_10;
      struct MPU6050_EXT_SENS_DATA_11
      {
        uint8_t         EXT_SENS_DATA_11:8;
      }               EXT_SENS_DATA_11;
      struct MPU6050_EXT_SENS_DATA_12
      {
        uint8_t         EXT_SENS_DATA_12:8;
      }               EXT_SENS_DATA_12;
      struct MPU6050_EXT_SENS_DATA_13
      {
        uint8_t         EXT_SENS_DATA_13:8;
      }               EXT_SENS_DATA_13;
      struct MPU6050_EXT_SENS_DATA_14
      {
        uint8_t         EXT_SENS_DATA_14:8;
      }               EXT_SENS_DATA_14;
      struct MPU6050_EXT_SENS_DATA_15
      {
        uint8_t         EXT_SENS_DATA_15:8;
      }               EXT_SENS_DATA_15;
      struct MPU6050_EXT_SENS_DATA_16
      {
        uint8_t         EXT_SENS_DATA_16:8;
      }               EXT_SENS_DATA_16;
      struct MPU6050_EXT_SENS_DATA_17
      {
        uint8_t         EXT_SENS_DATA_17:8;
      }               EXT_SENS_DATA_17;
      struct MPU6050_EXT_SENS_DATA_18
      {
        uint8_t         EXT_SENS_DATA_18:8;
      }               EXT_SENS_DATA_18;
      struct MPU6050_EXT_SENS_DATA_19
      {
        uint8_t         EXT_SENS_DATA_19:8;
      }               EXT_SENS_DATA_19;
      struct MPU6050_EXT_SENS_DATA_20
      {
        uint8_t         EXT_SENS_DATA_20:8;
      }               EXT_SENS_DATA_20;
      struct MPU6050_EXT_SENS_DATA_21
      {
        uint8_t         EXT_SENS_DATA_21:8;
      }               EXT_SENS_DATA_21;
      struct MPU6050_EXT_SENS_DATA_22
      {
        uint8_t         EXT_SENS_DATA_22:8;
      }               EXT_SENS_DATA_22;
      struct MPU6050_EXT_SENS_DATA_23
      {
        uint8_t         EXT_SENS_DATA_23:8;
      }               EXT_SENS_DATA_23;
      struct MPU6050_I2C_SLV0_DO
      {
        uint8_t         I2C_SLV0_DO:8;
      }               I2C_SLV0_DO;
      struct MPU6050_I2C_SLV1_DO
      {
        uint8_t         I2C_SLV1_DO:8;
      }               I2C_SLV1_DO;
      struct MPU6050_I2C_SLV2_DO
      {
        uint8_t         I2C_SLV2_DO:8;
      }               I2C_SLV2_DO;
      struct MPU6050_I2C_SLV3_DO
      {
        uint8_t       I2C_SLV3_DO:8;
      }               I2C_SLV3_DO;
      struct MPU6050_I2C_MST_DELAY_CTRL
      {
        uint8_t         I2C_SLV0_DLY_EN:1;
        uint8_t         I2C_SLV1_DLY_EN:1;
        uint8_t         I2C_SLV2_DLY_EN:1;
        uint8_t         I2C_SLV3_DLY_EN:1;
        uint8_t         I2C_SLV4_DLY_EN:1;
        uint8_t         reserved:2;
        uint8_t         DELAY_ES_SHADOW:1;
      }               I2C_MST_DELAY_CTRL;
      struct MPU6050_SIGNAL_PATH_RESET
      {
        uint8_t         TEMP_RESET:1;
        uint8_t         ACCCEL_RESET:1;
        uint8_t         GYRO_RESET:1;
        uint8_t         reserved:5;
      }               SIGNAL_PATH_RESET;
      struct MPU6050_USER_CTRL
      {
        uint8_t         SIG_COND_RESET:1;
        uint8_t         I2C_MST_RESET:1;
        uint8_t         FIFO_RESET:1;
        uint8_t         reserved0:1;
        uint8_t         I2C_IF_DIS:1;
        uint8_t         I2C_MST_EN:1;
        uint8_t         FIFO_EN:1;
        uint8_t         reserved1:1;
      }               USER_CTRL;
      struct MPU6050_PWR_MGMT_1
      {
        uint8_t         CLKSEL:3;
        uint8_t         TEMP_DIS:1;
        uint8_t         reserved:1;
        uint8_t         CYCLE:1;
        uint8_t         SLEEP:1;
        uint8_t         DEVICE_RESET:1;
      }               PWR_MGMT_1;
      struct MPU6050_PWR_MGMT_2
      {
        uint8_t         STBY_ZG:1;
        uint8_t         STBY_YG:1;
        uint8_t         STBY_XG:1;
        uint8_t         STBY_ZA:1;
        uint8_t         STBY_YA:1;
        uint8_t         STBY_XA:1;
        uint8_t         LP_WAKE_CTRL:2;
      }               PWR_MGMT_2;
      struct MPU6050_FIFO_COUNTH
      {
        uint8_t         FIFO_CNT:8;
      }               FIFO_COUNTH;
      struct MPU6050_FIFO_COUNTL
      {
        uint8_t         FIFO_CNT:8;
      }               FIFO_COUNTL;
      struct MPU6050_FIFO_R_W
      {
        uint8_t         FIFO_DATA:8;
      }               FIFO_R_W;
      struct MPU6050_WHO_AM_I
      {
        uint8_t         WHOAMI:8;
      }               WHO_AM_I;
    }       data;
  }       field;
  uint8_t value[1+1];
};


#endif /* MPU6050_H_ */
