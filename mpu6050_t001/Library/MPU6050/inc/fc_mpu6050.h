/*
 * mpu6050.h
 *
 *  Created on: Mar 5, 2022
 *      Author: Furkan
 */

#ifndef INC_FC_MPU6050_H_
#define INC_FC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include "i2cbitbang.h"

typedef enum {

	MPU6050_GYRO_SENS_RANGE_250,
	MPU6050_GYRO_SENS_RANGE_500,
	MPU6050_GYRO_SENS_RANGE_1000,
	MPU6050_GYRO_SENS_RANGE_2000

}GyroSensivityRange;

typedef enum {

	MPU6050_ACCEL_SENS_RANGE_2G,
	MPU6050_ACCEL_SENS_RANGE_4G,
	MPU6050_ACCEL_SENS_RANGE_8G,
	MPU6050_ACCEL_SENS_RANGE_16G

}AccelSensivityRange;

typedef struct {

	double   data[3];
	int16_t  cal_data[3];
	int16_t  register_data[3];
	double   sensivity;
	double   elapsed_time;
	uint32_t previous_time;
	uint32_t current_time;

}GyroTypeDef, GYRO, *HGYRO;

typedef struct {

	double 	data[3];
	int16_t cal_data[3];
	int16_t register_data[3];
	int16_t sensivity;

}AccelTypeDef, ACCEL, *HACCEL;

typedef struct {

	AccelTypeDef accel;
	GyroTypeDef gyro;

	double pitch;
	double roll;

}MPU6050, *HMPU6050;

/*!< Macro Definitions */
//-----------------------//

#define MPU6050_I2C_ADDR 			0xD0

#define MPU6050_I2C_ADD_WRITE		0xD0
#define MPU6050_I2C_ADD_READ		0xD1
#define MPU6050_WHO_AM_I			0x75
#define MPU6050_WHO_AM_I_VAL		0x68
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C

/*Accelerometer register address*/

#define MPU6050_ACCEL_XOUT_H 		0x3B
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_PWR_MGMT_1			0x6B
#define EARTH_GRAVITY               9.800665F
#define SENSORS_RADS_TO_DPS         57.29577951308232F
#define COMPLEMENTARY_RATE			0.9996F
#define MILISECOND_TO_SECOND 		0.001F
#define CALIBRATION_SIZE			200

/*!< Function Declarations */
//--------------------------//

uint8_t mpu6050_who_am(HI2C i2c);

uint8_t mpu6050_init(HI2C i2c);

uint8_t select_gyro_sensivity(HI2C i2c, HGYRO gyro, GyroSensivityRange sensivity);

uint8_t select_accel_sensivity(HI2C i2c, HACCEL accel, AccelSensivityRange sensivity);

uint8_t set_reg_accel(HI2C i2c, HACCEL accel);

uint8_t set_reg_gyro(HI2C i2c, HGYRO gyro);

void mpu6050_calibration(HI2C i2c, HMPU6050 mpu6050);

void set_degree_accel(HACCEL accel);

void set_degree_gyro(HGYRO gyro);

#endif /* INC_FC_MPU6050_H_ */
