/*
 * Author : Roche Christopher
 * email  : rochextopher@gmail.com
 *
 */

/*
 *
	MIT License

	Copyright (c) 2024 Roche Christopher

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 *
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"

/*
 * ======= Register Addresses
 */
#define MPU6050_CONFIG_REG 0x1A
#define MPU6050_GYRO_CONFIG_REG 0x1B
#define MPU6050_ACCEL_CONFIG_REG 0x1C
#define MPU6050_SMPLRT_DIV_REG 0x19
#define MPU6050_INT_EN_REG 0x38
#define MPU6050_INT_PIN_CFG_REG 0x37
#define MPU6050_INT_STATUS_REG 0x3A
#define MPU6050_POWER_MGMT_REG 0x6B
#define MPU6050_FIFO_EN 0x23
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_FIFO_COUNTH 0x72
#define MPU6050_FIFO_COUNTL 0x73
#define MPU6050_FIFO_BUFFER 0x74

/* Gyro data addresses */
#define MPU6050_GYRO_X_HIGH 0x43
#define MPU6050_GYRO_X_LOW 0x44
#define MPU6050_GYRO_Y_HIGH 0x45
#define MPU6050_GYRO_Y_LOW 0x46
#define MPU6050_GYRO_Z_HIGH 0x47
#define MPU6050_GYRO_Z_LOW 0x48

/*
 * ======= Register Addresses End
 */


extern const uint16_t mpu6050_dev_addr;
extern int gyroscope_output_rate;

extern I2C_HandleTypeDef *mpu6050_i2c_bus;

typedef struct GyroData{
	int16_t x;
	int16_t y;
	int16_t z;
} GyroData;

/* Reading and writing in registers */

int write_to_register(uint16_t reg_addr, uint8_t *reg_value);
int read_from_register(uint16_t reg_addr, uint8_t *reg_value);


/* configuration functions */

int set_configuration(uint8_t *config);
int set_gyroscope_configuration(uint8_t *gyro_config);
int set_sampling_rate(int sampling_rate); // sampling rate in hertz
int set_smplrt_div(uint8_t *smplrt_value);
int set_interrupt_enable(uint8_t *interrupt_enable_value);
int set_interrupt_pin_configuration(uint8_t *interrupt_pin_config);
int write_to_power_mgmt(uint8_t *pwr_mgmt_reg_value);
int set_user_control(uint8_t *user_control_value);
int set_fifo_enable(uint8_t *fifo_enable_value);
uint16_t get_fifo_count();
int read_from_fifo(uint8_t *buffer_data);
int16_t get_unified_data_from_fifo();

/* Reading from the IMU functions */

int get_interrupt_status(uint8_t *interrupt_status);
int get_gyro_x(int16_t *gyro_x);
int get_gyro_y(int16_t *gyro_y);
int get_gyro_z(int16_t *gyro_z);


#endif

