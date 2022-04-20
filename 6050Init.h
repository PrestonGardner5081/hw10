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
#define APB_CLOCK 250000000

#define ROUND_DIVISION(x, y) (((x) + (y) / 2) / (y))

union uint16_to_2uint8
{
  struct uint16_to_2uint8_field
  {
    uint8_t L; /* Little Endian byte order means that the least significant byte goes in the lowest address */
    uint8_t H;
  } field;
  uint16_t unsigned_value;
  int16_t signed_value;
};

struct calibration_data
{
  float scale;
  float offset_x;
  float offset_y;
  float offset_z;
};

struct spatial_data
{
  union uint16_to_2uint8 ACCEL_XOUT;
  union uint16_to_2uint8 ACCEL_YOUT;
  union uint16_to_2uint8 ACCEL_ZOUT;
  union uint16_to_2uint8 GYRO_XOUT;
  union uint16_to_2uint8 GYRO_YOUT;
  union uint16_to_2uint8 GYRO_ZOUT;
};

int init_gyro(struct io_peripherals *io, struct calibration_data *calibration_accelerometer,
              struct calibration_data *calibration_gyroscope,
              struct calibration_data *calibration_magnetometer);

void read_accelerometer_gyroscope(
    struct calibration_data *calibration_accelerometer,
    struct calibration_data *calibration_gyroscope,
    struct spatial_data *sp_data,
    volatile struct bsc_register *bsc);

void calibrate_accelerometer_and_gyroscope(
    struct calibration_data *calibration_accelerometer,
    struct calibration_data *calibration_gyroscope,
    volatile struct bsc_register *bsc);