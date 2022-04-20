/**************************************************
* CMPEN 473, Spring 2022, Penn State University
*
* Sample Program hw0a7imu2i2c6050   ***  ***
*   - for MPU-6050 IMU chip, accelerometer and gyroscope reading  examples
*   - accelerometer X, Y, Z, and gyroscope X, Y, Z examples
*   - I2C interface with Raspberry Pi 4 computer and MPU-6050 IMU chip
*   - I2C device address: "i2cdetect -y 1" command yields 0x68
*   - hit any key to quit
*
*           Author: steveb
*       Created on: March 04, 2022
*  Last updated on: March 07, 2022
*       Updated by: K.Choi
*
* Genetic MPU-6050 IMU board connected to I2C pins of Raspberry Pi 4 computer
* (low cost MPU-6050 IMU board available on Amazon.com, $2.00/board - 2022)
*
***************************************************/

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include "import_registers.h"
#include "cm.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "bsc.h"
#include "pwm.h"
#include "enable_pwm_clock.h"
#include "io_peripherals.h"
#include "FIFO.h"
#include "MPU6050.h"
#include "6050Init.h"

void read_MPU6050_registers(                                   /* read a register */
                            uint8_t I2C_address,               /* the address of the I2C device to talk to */
                            MPU6050_REGISTER register_address, /* the address to read from */
                            uint8_t *read_data,                /* the data read from the SPI device */
                            size_t data_length,                /* the length of data to send/receive */
                            volatile struct bsc_register *bsc) /* the BSC address */
{
  bsc->S.field.DONE = 1;
  bsc->A.field.ADDR = I2C_address;
  bsc->C.field.READ = 0;
  bsc->DLEN.field.DLEN = 1;
  bsc->FIFO.value = register_address;
  bsc->C.field.ST = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep(100);
  }
  bsc->S.field.DONE = 1;
  bsc->A.field.ADDR = I2C_address;
  bsc->C.field.READ = 1;
  bsc->DLEN.field.DLEN = data_length;
  bsc->C.field.ST = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep(100);
  }

  while (data_length > 0)
  {
    *read_data = bsc->FIFO.field.DATA;

    read_data++;
    data_length--;
  }

  return;
}

union MPU6050_transaction_field_data read_MPU6050_register(                                   /* read a register, returning the read value */
                                                           uint8_t I2C_address,               /* the address of the I2C device to talk to */
                                                           MPU6050_REGISTER register_address, /* the address to read from */
                                                           volatile struct bsc_register *bsc) /* the BSC address */
{
  union MPU6050_transaction transaction;

  read_MPU6050_registers(I2C_address, register_address, &(transaction.value[1]), 1, bsc);

  return transaction.field.data;
}

void write_MPU6050_register(                                            /* write a register */
                            uint8_t I2C_address,                        /* the address of the I2C device to talk to */
                            MPU6050_REGISTER register_address,          /* the address to read from */
                            union MPU6050_transaction_field_data value, /* the value to write */
                            volatile struct bsc_register *bsc)          /* the BSC address */
{
  union MPU6050_transaction transaction;

  transaction.field.data = value;
  bsc->S.field.DONE = 1;
  bsc->A.field.ADDR = I2C_address;
  bsc->C.field.READ = 0;
  bsc->DLEN.field.DLEN = 2;
  bsc->FIFO.value = register_address;
  bsc->FIFO.value = transaction.value[1];
  bsc->C.field.ST = 1;
  while (bsc->S.field.DONE == 0)
  {
    usleep(100);
  }

  return;
}

void calibrate_accelerometer_and_gyroscope(
    struct calibration_data *calibration_accelerometer,
    struct calibration_data *calibration_gyroscope,
    volatile struct bsc_register *bsc)
{
  union MPU6050_transaction_field_data transaction;
  uint8_t data_block_fifo_count[2];
  union uint16_to_2uint8 reconstructor;
  uint16_t ii;
  uint16_t packet_count;
  int32_t gyro_bias_x;
  int32_t gyro_bias_y;
  int32_t gyro_bias_z;
  int32_t accel_bias_x;
  int32_t accel_bias_y;
  int32_t accel_bias_z;
  uint8_t data_block_fifo_packet[12];
  union uint16_to_2uint8 reconstructor_accel_x;
  union uint16_to_2uint8 reconstructor_accel_y;
  union uint16_to_2uint8 reconstructor_accel_z;
  union uint16_to_2uint8 reconstructor_gyro_x;
  union uint16_to_2uint8 reconstructor_gyro_y;
  union uint16_to_2uint8 reconstructor_gyro_z;

  // reset device
  transaction.PWR_MGMT_1.CLKSEL = 0;
  transaction.PWR_MGMT_1.TEMP_DIS = 0;
  transaction.PWR_MGMT_1.reserved = 0;
  transaction.PWR_MGMT_1.CYCLE = 0;
  transaction.PWR_MGMT_1.SLEEP = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 1;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc);
  usleep(100000);

  // get stable time source; auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator
  transaction.PWR_MGMT_1.CLKSEL = 1;
  transaction.PWR_MGMT_1.TEMP_DIS = 0;
  transaction.PWR_MGMT_1.reserved = 0;
  transaction.PWR_MGMT_1.CYCLE = 0;
  transaction.PWR_MGMT_1.SLEEP = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc);
  transaction.PWR_MGMT_2.STBY_ZG = 0;
  transaction.PWR_MGMT_2.STBY_YG = 0;
  transaction.PWR_MGMT_2.STBY_XG = 0;
  transaction.PWR_MGMT_2.STBY_ZA = 0;
  transaction.PWR_MGMT_2.STBY_YA = 0;
  transaction.PWR_MGMT_2.STBY_XA = 0;
  transaction.PWR_MGMT_2.LP_WAKE_CTRL = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_2, transaction, bsc);
  usleep(200000);

  // configure device for bias calculation
  transaction.INT_ENABLE.DATA_RDY_EN = 0; // disable all interrupts
  transaction.INT_ENABLE.reserved0 = 0;
  transaction.INT_ENABLE.I2C_MST_INT_EN = 0;
  transaction.INT_ENABLE.FIFO_OFLOW_EN = 0;
  transaction.INT_ENABLE.reserved1 = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_INT_ENABLE, transaction, bsc);
  transaction.FIFO_EN.SLV0_FIFO_EN = 0; // disable FIFO
  transaction.FIFO_EN.SLV1_FIFO_EN = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 0;
  transaction.FIFO_EN.ZG_FIFO_EN = 0;
  transaction.FIFO_EN.YG_FIFO_EN = 0;
  transaction.FIFO_EN.XG_FIFO_EN = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc);
  transaction.PWR_MGMT_1.CLKSEL = 0; // turn on internal clock source
  transaction.PWR_MGMT_1.TEMP_DIS = 0;
  transaction.PWR_MGMT_1.reserved = 0;
  transaction.PWR_MGMT_1.CYCLE = 0;
  transaction.PWR_MGMT_1.SLEEP = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc);
  transaction.I2C_MST_CTRL.I2C_MST_CLK = 0; // disable I2C master
  transaction.I2C_MST_CTRL.I2C_MST_P_NSR = 0;
  transaction.I2C_MST_CTRL.SLV_3_FIFO_EN = 0;
  transaction.I2C_MST_CTRL.WAIT_FOR_ES = 0;
  transaction.I2C_MST_CTRL.MULT_MST_EN = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_I2C_MST_CTRL, transaction, bsc);
  transaction.USER_CTRL.SIG_COND_RESET = 0; // disable FIFO and I2C master modes
  transaction.USER_CTRL.I2C_MST_RESET = 0;
  transaction.USER_CTRL.FIFO_RESET = 0;
  transaction.USER_CTRL.reserved0 = 0;
  transaction.USER_CTRL.I2C_IF_DIS = 0;
  transaction.USER_CTRL.I2C_MST_EN = 0;
  transaction.USER_CTRL.FIFO_EN = 0;
  transaction.USER_CTRL.reserved1 = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc);
  transaction.USER_CTRL.SIG_COND_RESET = 0; // reset FIFO and DMP
  transaction.USER_CTRL.I2C_MST_RESET = 0;
  transaction.USER_CTRL.FIFO_RESET = 1;
  transaction.USER_CTRL.reserved0 = 0;
  transaction.USER_CTRL.I2C_IF_DIS = 0;
  transaction.USER_CTRL.I2C_MST_EN = 0;
  transaction.USER_CTRL.FIFO_EN = 0;
  transaction.USER_CTRL.reserved1 = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc);
  usleep(15000);

  // configure MPU6050 gyro and accelerometer for bias calculation
  transaction.CONFIG.DLPF_CFG = 1; // set low-pass filter to 188Hz
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.reserved = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_CONFIG, transaction, bsc);
  transaction.SMPLRT_DIV.SMPLRT_DIV = 0; // set sample rate to 1kHz
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_SMPLRT_DIV, transaction, bsc);
  transaction.GYRO_CONFIG.reserved0 = 0; // set gyro full-scale to 250dps, maximum sensitivity
  transaction.GYRO_CONFIG.FS_SEL = 0;
  transaction.GYRO_CONFIG.reserved1 = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, transaction, bsc);
  transaction.ACCEL_CONFIG.reserved = 0; // set accelerometer full-scale to 2g, maximum sensitivity
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.ZA_ST = 0;
  transaction.ACCEL_CONFIG.YA_ST = 0;
  transaction.ACCEL_CONFIG.XA_ST = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, transaction, bsc);

  calibration_accelerometer->scale = 2.0 / 32768.0; // measurement scale/signed numeric range
  calibration_accelerometer->offset_x = 0;
  calibration_accelerometer->offset_y = 0;
  calibration_accelerometer->offset_z = 0;

  calibration_gyroscope->scale = 250.0 / 32768.0;
  calibration_gyroscope->offset_x = 0;
  calibration_gyroscope->offset_y = 0;
  calibration_gyroscope->offset_z = 0;

  // configure FIFO to capture accelerometer and gyro data for bias calculation
  transaction.USER_CTRL.SIG_COND_RESET = 0; // enable FIFO
  transaction.USER_CTRL.I2C_MST_RESET = 0;
  transaction.USER_CTRL.FIFO_RESET = 0;
  transaction.USER_CTRL.reserved0 = 0;
  transaction.USER_CTRL.I2C_IF_DIS = 0;
  transaction.USER_CTRL.I2C_MST_EN = 0;
  transaction.USER_CTRL.FIFO_EN = 1;
  transaction.USER_CTRL.reserved1 = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc);
  transaction.FIFO_EN.SLV0_FIFO_EN = 0; // enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU6050)
  transaction.FIFO_EN.SLV1_FIFO_EN = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 1;
  transaction.FIFO_EN.ZG_FIFO_EN = 1;
  transaction.FIFO_EN.YG_FIFO_EN = 1;
  transaction.FIFO_EN.XG_FIFO_EN = 1;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc);
  usleep(40000); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // at end of sample accumulation, turn off FIFO sensor read
  transaction.FIFO_EN.SLV0_FIFO_EN = 0; // disable gyro and accelerometer sensors for FIFO
  transaction.FIFO_EN.SLV1_FIFO_EN = 0;
  transaction.FIFO_EN.SLV2_FIFO_EN = 0;
  transaction.FIFO_EN.ACCEL_FIFO_EN = 0;
  transaction.FIFO_EN.ZG_FIFO_EN = 0;
  transaction.FIFO_EN.YG_FIFO_EN = 0;
  transaction.FIFO_EN.XG_FIFO_EN = 0;
  transaction.FIFO_EN.TEMP_FIFO_EN = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_EN, transaction, bsc);
  read_MPU6050_registers(MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_COUNTH, data_block_fifo_count, sizeof(data_block_fifo_count), bsc); // read FIFO sample count
  reconstructor.field.H = data_block_fifo_count[0];
  reconstructor.field.L = data_block_fifo_count[1];
  packet_count = reconstructor.unsigned_value / 12; // how many sets of full gyro and accelerometer data for averaging

  accel_bias_x = 0;
  accel_bias_y = 0;
  accel_bias_z = 0;
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;
  for (ii = 0; ii < packet_count; ii++)
  {
    read_MPU6050_registers(MPU6050_ADDRESS, MPU6050_REGISTER_FIFO_R_W, data_block_fifo_packet, sizeof(data_block_fifo_packet), bsc); // read data for averaging

    reconstructor_accel_x.field.H = data_block_fifo_packet[0];
    reconstructor_accel_x.field.L = data_block_fifo_packet[1];
    reconstructor_accel_y.field.H = data_block_fifo_packet[2];
    reconstructor_accel_y.field.L = data_block_fifo_packet[3];
    reconstructor_accel_z.field.H = data_block_fifo_packet[4];
    reconstructor_accel_z.field.L = data_block_fifo_packet[5];
    reconstructor_gyro_x.field.H = data_block_fifo_packet[6];
    reconstructor_gyro_x.field.L = data_block_fifo_packet[7];
    reconstructor_gyro_y.field.H = data_block_fifo_packet[8];
    reconstructor_gyro_y.field.L = data_block_fifo_packet[9];
    reconstructor_gyro_z.field.H = data_block_fifo_packet[10];
    reconstructor_gyro_z.field.L = data_block_fifo_packet[11];

    accel_bias_x += reconstructor_accel_x.signed_value; // sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias_y += reconstructor_accel_y.signed_value;
    accel_bias_z += reconstructor_accel_z.signed_value;
    gyro_bias_x += reconstructor_gyro_x.signed_value;
    gyro_bias_y += reconstructor_gyro_y.signed_value;
    gyro_bias_z += reconstructor_gyro_z.signed_value;
  }
  accel_bias_x /= (int32_t)packet_count;
  accel_bias_y /= (int32_t)packet_count;
  accel_bias_z /= (int32_t)packet_count;
  gyro_bias_x /= (int32_t)packet_count;
  gyro_bias_y /= (int32_t)packet_count;
  gyro_bias_z /= (int32_t)packet_count;

  if (accel_bias_z > 0) // remove gravity from the z-axis accelerometer bias calculation
  {
    accel_bias_z -= (int32_t)(1.0 / calibration_accelerometer->scale);
  }
  else
  {
    accel_bias_z += (int32_t)(1.0 / calibration_accelerometer->scale);
  }

  // the code that this is based off of tried to push the bias calculation values to hardware correction registers
  // these registers do not appear to be functioning, so rely on software offset correction

  // output scaled gyro biases
  calibration_gyroscope->offset_x = ((float)gyro_bias_x) * calibration_gyroscope->scale;
  calibration_gyroscope->offset_y = ((float)gyro_bias_y) * calibration_gyroscope->scale;
  calibration_gyroscope->offset_z = ((float)gyro_bias_z) * calibration_gyroscope->scale;

  // output scaled accelerometer biases
  calibration_accelerometer->offset_x = ((float)accel_bias_x) * calibration_accelerometer->scale;
  calibration_accelerometer->offset_y = ((float)accel_bias_y) * calibration_accelerometer->scale;
  calibration_accelerometer->offset_z = ((float)accel_bias_z) * calibration_accelerometer->scale;

  return;
}

void initialize_accelerometer_and_gyroscope(
    struct calibration_data *calibration_accelerometer,
    struct calibration_data *calibration_gyroscope,
    volatile struct bsc_register *bsc)
{
  union MPU6050_transaction_field_data transaction;

  /* print WHO_AM_I */
  printf("accel WHOAMI (0x68) = 0x%2.2X\n",
         read_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_WHO_AM_I, bsc).WHO_AM_I.WHOAMI);

  // based off https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

  calibrate_accelerometer_and_gyroscope(calibration_accelerometer, calibration_gyroscope, bsc);

  // reset MPU9205
  transaction.PWR_MGMT_1.CLKSEL = 0;
  transaction.PWR_MGMT_1.TEMP_DIS = 0;
  transaction.PWR_MGMT_1.reserved = 0;
  transaction.PWR_MGMT_1.CYCLE = 0;
  transaction.PWR_MGMT_1.SLEEP = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 1;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc);
  usleep(1000); // wait for all registers to reset

  // clock source
  transaction.PWR_MGMT_1.CLKSEL = 1;
  transaction.PWR_MGMT_1.TEMP_DIS = 0;
  transaction.PWR_MGMT_1.reserved = 0;
  transaction.PWR_MGMT_1.CYCLE = 0;
  transaction.PWR_MGMT_1.SLEEP = 0;
  transaction.PWR_MGMT_1.DEVICE_RESET = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc);

  // enable acc & gyro
  transaction.PWR_MGMT_2.STBY_ZG = 0;
  transaction.PWR_MGMT_2.STBY_YG = 0;
  transaction.PWR_MGMT_2.STBY_XG = 0;
  transaction.PWR_MGMT_2.STBY_ZA = 0;
  transaction.PWR_MGMT_2.STBY_YA = 0;
  transaction.PWR_MGMT_2.STBY_XA = 0;
  transaction.PWR_MGMT_2.LP_WAKE_CTRL = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, transaction, bsc);

  // use DLPF set gyro bandwidth 184Hz, temperature bandwidth 188Hz
  transaction.CONFIG.DLPF_CFG = 1;
  transaction.CONFIG.EXT_SYNC_SET = 0;
  transaction.CONFIG.reserved = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_CONFIG, transaction, bsc);

  // +-250dps
  transaction.GYRO_CONFIG.reserved0 = 0;
  transaction.GYRO_CONFIG.FS_SEL = 0;
  transaction.GYRO_CONFIG.reserved1 = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, transaction, bsc);

  // +-2G
  transaction.ACCEL_CONFIG.reserved = 0;
  transaction.ACCEL_CONFIG.ACCEL_FS_SEL = 0;
  transaction.ACCEL_CONFIG.ZA_ST = 0;
  transaction.ACCEL_CONFIG.YA_ST = 0;
  transaction.ACCEL_CONFIG.XA_ST = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, transaction, bsc);

  // force into I2C mode, disabling I2C master
  transaction.USER_CTRL.SIG_COND_RESET = 0;
  transaction.USER_CTRL.I2C_MST_RESET = 0;
  transaction.USER_CTRL.FIFO_RESET = 0;
  transaction.USER_CTRL.reserved0 = 0;
  transaction.USER_CTRL.I2C_IF_DIS = 0;
  transaction.USER_CTRL.I2C_MST_EN = 0;
  transaction.USER_CTRL.FIFO_EN = 0;
  transaction.USER_CTRL.reserved1 = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_USER_CTRL, transaction, bsc);

  // enable bypass mode
  transaction.INT_PIN_CFG.reserved = 0;
  transaction.INT_PIN_CFG.I2C_BYPASS_EN = 1;
  transaction.INT_PIN_CFG.FSYNC_INT_EN = 0;
  transaction.INT_PIN_CFG.FSYNC_INT_LEVEL = 0;
  transaction.INT_PIN_CFG.INT_RD_CLEAR = 0;
  transaction.INT_PIN_CFG.LATCH_INT_EN = 0;
  transaction.INT_PIN_CFG.INT_OPEN = 0;
  transaction.INT_PIN_CFG.INT_LEVEL = 0;
  write_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_INT_PIN_CFG, transaction, bsc);

  return;
}

void read_accelerometer_gyroscope(
    struct calibration_data *calibration_accelerometer,
    struct calibration_data *calibration_gyroscope,
    struct spatial_data *sp_data,
    volatile struct bsc_register *bsc)
{
  uint8_t data_block[6 + 2 + 6];

  /*
   * poll the interrupt status register and it tells you when it is done
   * once it is done, read the data registers
   */
  do
  {
    usleep(1000);
  } while (read_MPU6050_register(MPU6050_ADDRESS, MPU6050_REGISTER_INT_STATUS, bsc).INT_STATUS.DATA_RDY_INT == 0);

  // read the accelerometer values
  read_MPU6050_registers(MPU6050_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H, data_block, sizeof(data_block), bsc);
  sp_data->ACCEL_XOUT.field.H = data_block[0];
  sp_data->ACCEL_XOUT.field.L = data_block[1];
  sp_data->ACCEL_YOUT.field.H = data_block[2];
  sp_data->ACCEL_YOUT.field.L = data_block[3];
  sp_data->ACCEL_ZOUT.field.H = data_block[4];
  sp_data->ACCEL_ZOUT.field.L = data_block[5];
  // TEMP_OUT.field.H = data_block[6];
  // TEMP_OUT.field.L = data_block[7];
  sp_data->GYRO_XOUT.field.H = data_block[8];
  sp_data->GYRO_XOUT.field.L = data_block[9];
  sp_data->GYRO_YOUT.field.H = data_block[10];
  sp_data->GYRO_YOUT.field.L = data_block[11];
  sp_data->GYRO_ZOUT.field.H = data_block[12];
  sp_data->GYRO_ZOUT.field.L = data_block[13];

  // printf("Gyro X: %.2f deg\ty=%.2f deg\tz=%.2f deg\n",
  //        sp_data->GYRO_XOUT.signed_value * calibration_gyroscope->scale - calibration_gyroscope->offset_x,
  //        sp_data->GYRO_YOUT.signed_value * calibration_gyroscope->scale - calibration_gyroscope->offset_y,
  //        sp_data->GYRO_ZOUT.signed_value * calibration_gyroscope->scale - calibration_gyroscope->offset_z);

  // printf("Accel X: %.2f m/s^2\ty=%.2f m/s^2\tz=%.2f m/s^2\n",
  //        (sp_data->ACCEL_XOUT.signed_value * calibration_accelerometer->scale - calibration_accelerometer->offset_x) * 9.81,
  //        (sp_data->ACCEL_YOUT.signed_value * calibration_accelerometer->scale - calibration_accelerometer->offset_y) * 9.81,
  //        (sp_data->ACCEL_ZOUT.signed_value * calibration_accelerometer->scale - calibration_accelerometer->offset_z) * 9.81);

  return;
}

int init_gyro(struct io_peripherals *io, struct calibration_data *calibration_accelerometer,
              struct calibration_data *calibration_gyroscope,
              struct calibration_data *calibration_magnetometer)
{

  /* set the pin function to alternate function 0 for GPIO02 (I2C1, SDA) */
  /* set the pin function to alternate function 0 for GPIO03 (I2C1, SCL) */
  io->gpio->GPFSEL0.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
  io->gpio->GPFSEL0.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

  /* configure the I2C interface */
  io->bsc->DIV.field.CDIV = (PERIPHERAL_CLOCK * 10) / 400000;
  io->bsc->DEL.field.REDL = 0x30;
  io->bsc->DEL.field.FEDL = 0x30;
  io->bsc->CLKT.field.TOUT = 0x40;
  io->bsc->C.field.INTD = 0;
  io->bsc->C.field.INTT = 0;
  io->bsc->C.field.INTR = 0;
  io->bsc->C.field.I2CEN = 1;
  io->bsc->C.field.CLEAR = 1;

  initialize_accelerometer_and_gyroscope(calibration_accelerometer, calibration_gyroscope, io->bsc);
  calibrate_accelerometer_and_gyroscope(calibration_accelerometer, calibration_gyroscope, io->bsc);

  return 0;
}

// int main(void)
// {
//   struct io_peripherals *io;
//   struct calibration_data calibration_accelerometer;
//   struct calibration_data calibration_gyroscope;
//   struct calibration_data calibration_magnetometer;
//   struct spatial_data sd;
//   sd.ACCEL_XOUT.field.H, sd.ACCEL_YOUT.field.H, sd.ACCEL_ZOUT.field.H, sd.GYRO_XOUT.field.H, sd.GYRO_YOUT.field.H, sd.GYRO_ZOUT.field.H, sd.ACCEL_XOUT.field.L, sd.ACCEL_YOUT.field.L, sd.ACCEL_ZOUT.field.L, sd.GYRO_XOUT.field.L, sd.GYRO_YOUT.field.L, sd.GYRO_ZOUT.field.L = (uint8_t)0;

//   io = import_registers();

//   init_gyro(io, &calibration_accelerometer, &calibration_gyroscope, &calibration_magnetometer);

//   do
//   {
//     sd.ACCEL_XOUT.field.H, sd.ACCEL_YOUT.field.H, sd.ACCEL_ZOUT.field.H, sd.GYRO_XOUT.field.H, sd.GYRO_YOUT.field.H, sd.GYRO_ZOUT.field.H, sd.ACCEL_XOUT.field.L, sd.ACCEL_YOUT.field.L, sd.ACCEL_ZOUT.field.L, sd.GYRO_XOUT.field.L, sd.GYRO_YOUT.field.L, sd.GYRO_ZOUT.field.L = (uint8_t)0;
//     read_accelerometer_gyroscope(&calibration_accelerometer, &calibration_gyroscope, &sd, io->bsc);
//     printf("\n");
//   } while (!wait_key(100, 0));
// }