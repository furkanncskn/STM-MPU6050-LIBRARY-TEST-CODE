/*
 * mpu6050.c
 *
 *  Created on: Mar 5, 2022
 *      Author: Furkan
 */

#include <fc_mpu6050.h>
#include <fc_utility.h>
#include "i2cbitbang.h"
#include "math.h"

#define PUBLIC
#define PRIVATE	static

/*!<   Typedef Enum Declarations  */
//--------------------------------//

typedef enum {

	MPU6050_AFS_SEL_0   = 0x00,
	MPU6050_AFS_SEL_1   = 0x08,
	MPU6050_AFS_SEL_2   = 0x10,
	MPU6050_AFS_SEL_3   = 0x18,

}GyroAfsSel;

typedef enum {

	MPU6050_FS_SEL_0    = 0x00,
	MPU6050_FS_SEL_1    = 0x08,
	MPU6050_FS_SEL_2    = 0x10,
	MPU6050_FS_SEL_3    = 0x18,

}AccelFsSel;


/*!< Static Function Declarations */
//--------------------------------//

PRIVATE uint8_t set_gyro_sensivity(HI2C i2cPort, HGYRO gyro, double sensivity, uint8_t afs_sel);

PRIVATE uint8_t set_accel_sensivity(HI2C i2cPort, HACCEL accel, int16_t sensivity, uint8_t fs_sel);


/*!<     Function Definitions 	  */
//--------------------------------//

/**
 * @brief MPU6050 sensörünün varlığını kontrol eder
 */
PUBLIC uint8_t mpu6050_who_am(HI2C i2cPort)
{
	uint8_t reading_value = i2cPort->readReg(MPU6050_WHO_AM_I);

	if(reading_value != MPU6050_WHO_AM_I_VAL)
		return FCFAIL;

	return FCSUCCESS;
}

/**
 *@brief Baslangic ayarlarini yapar
 *
 */
PUBLIC uint8_t mpu6050_init(HI2C i2cPort)
{
	if(mpu6050_who_am(i2cPort) != FCFAIL)
	{
		uint8_t data = 0x00;
		i2cPort->writeReg(MPU6050_PWR_MGMT_1, data);

		data = 0x07;
		i2cPort->writeReg(MPU6050_SMPLRT_DIV, data);

		return FCSUCCESS;
	}

	return FCFAIL;
}

/**
 * @brief Gyroscope sensivity ayarini secer
 *
 * @return Basari durumunda '0' Basarisizlik durumunda '1'
 */
PUBLIC uint8_t select_gyro_sensivity(HI2C i2cPort, HGYRO gyro, GyroSensivityRange sensivity)
{
	switch (sensivity)
	{
		case MPU6050_GYRO_SENS_RANGE_250:
			if(set_gyro_sensivity(i2cPort, gyro, 131., MPU6050_AFS_SEL_0))
				return FCFAIL;
			break;
		case MPU6050_GYRO_SENS_RANGE_500:
			if(set_gyro_sensivity(i2cPort, gyro, 65.5, MPU6050_AFS_SEL_1))
				return FCFAIL;
			break;
		case MPU6050_GYRO_SENS_RANGE_1000:
			if(set_gyro_sensivity(i2cPort, gyro, 32.8, MPU6050_AFS_SEL_2))
				return FCFAIL;
			break;
		case MPU6050_GYRO_SENS_RANGE_2000:
			if(set_gyro_sensivity(i2cPort, gyro, 16.4, MPU6050_AFS_SEL_3))
				return FCFAIL;
			break;
		default:
			if(set_gyro_sensivity(i2cPort, gyro, 16.4, MPU6050_AFS_SEL_3))
				return FCFAIL;
			break;
	}

	return FCSUCCESS;
}

/**
 * @brief Gyroscope'un sensivity degerini set eder
 *
 * @return Basari durumunda '0' Basarisizlikta '1'
 *
 */
PRIVATE uint8_t set_gyro_sensivity(HI2C i2cPort, HGYRO gyro, double sensivity, uint8_t afs_sel)
{
	i2cPort->writeReg(MPU6050_GYRO_CONFIG, afs_sel);

	if(i2cPort->getError() != I2CBB_ERROR_NONE)
		return FCFAIL;

	gyro->sensivity = sensivity;

	return FCSUCCESS;
}

/**
 * @brief Accelerometer sensivity ayarini secer
 *
 * @return Basari durumunda '0' Basarisizlik durumunda '1'
 */
PUBLIC uint8_t select_accel_sensivity(HI2C i2cPort, HACCEL accel, AccelSensivityRange sensivity)
{
	switch (sensivity)
	{
		case MPU6050_ACCEL_SENS_RANGE_2G:
			if(set_accel_sensivity(i2cPort, accel, 16384, MPU6050_FS_SEL_0))
				return FCFAIL;
			break;
		case MPU6050_ACCEL_SENS_RANGE_4G:
			if(set_accel_sensivity(i2cPort, accel, 8192, MPU6050_FS_SEL_1))
				return FCFAIL;
			break;
		case MPU6050_ACCEL_SENS_RANGE_8G:
			if(set_accel_sensivity(i2cPort, accel, 4096, MPU6050_FS_SEL_2))
				return FCFAIL;
			break;
		case MPU6050_ACCEL_SENS_RANGE_16G:
			if(set_accel_sensivity(i2cPort, accel, 2048, MPU6050_FS_SEL_3))
				return FCFAIL;
			break;
		default:
			if(set_accel_sensivity(i2cPort, accel, 16384, MPU6050_FS_SEL_0))
				return FCFAIL;
			break;
	}

	return FCSUCCESS;
}

/**
 * @brief Accelerometer'in sensivity degerini set eder
 *
 * @return Basari durumunda '0' Basarisizlikta '1'
 *
 */
PRIVATE uint8_t set_accel_sensivity(HI2C i2cPort, HACCEL accel, int16_t sensivity, uint8_t fs_sel)
{
	i2cPort->writeReg(MPU6050_ACCEL_CONFIG, fs_sel);

	if(i2cPort->getError() != I2CBB_ERROR_NONE)
		return FCFAIL;

	accel->sensivity = sensivity;

	return FCSUCCESS;
}

/**
 * @brief kalibrasyon değerlerini set eder
 *
 * @return Basari durumunda '0' Basarisizlik durumunda '1'
 */
PUBLIC void mpu6050_calibration(HI2C i2cPort, HMPU6050 mpu6050)
{
	int16_t temp_raw[5] = {0};

	uint16_t cnt = 0;
	for(uint16_t i = 0; i < CALIBRATION_SIZE; ++i)
	{
		set_reg_accel(i2cPort, &mpu6050->accel);
		if(i2cPort->getError() != I2CBB_ERROR_NONE)
			continue;

		HAL_Delay(5);

		set_reg_gyro(i2cPort, &mpu6050->gyro);
		if(i2cPort->getError() != I2CBB_ERROR_NONE)
			continue;

		temp_raw[0] += mpu6050->accel.register_data[0];
		temp_raw[1] += mpu6050->accel.register_data[1];

		temp_raw[2] += mpu6050->gyro.register_data[0];
		temp_raw[3] += mpu6050->gyro.register_data[1];
		temp_raw[4] += mpu6050->gyro.register_data[2];

		cnt++;
	}

	mpu6050->accel.cal_data[0] = temp_raw[0]  / cnt;
	mpu6050->accel.cal_data[1] = temp_raw[1]  / cnt;

	mpu6050->gyro.cal_data[0]  = temp_raw[2]  / cnt;
	mpu6050->gyro.cal_data[1]  = temp_raw[3]  / cnt;
	mpu6050->gyro.cal_data[2]  = temp_raw[4]  / cnt;
}

/**
 * @brief accelerometer'dan register degerlerini okur
 *
 * @return Basari durumunda '0' Basarisizlik durumunda '1'
 */
PUBLIC uint8_t set_reg_accel(HI2C i2cPort, HACCEL accel)
{
	uint8_t buffer[6] = {0};

	i2cPort->readData(MPU6050_ACCEL_XOUT_H, buffer, 6);

	if(i2cPort->getError() != I2CBB_ERROR_NONE)
		return FCFAIL;

	accel->register_data[0] = (int16_t)( buffer[0] << 8 | buffer[1] );
	accel->register_data[1] = (int16_t)( buffer[2] << 8 | buffer[3] );
	accel->register_data[2] = (int16_t)( buffer[4] << 8 | buffer[5] );

	accel->register_data[0] -= accel->cal_data[0];
	accel->register_data[1] -= accel->cal_data[1];

	return FCSUCCESS;
}

/**
 * @brief aci degerlerini set eder
 */
PUBLIC void set_degree_accel(HACCEL accel)
{
	accel->data[0] = (accel->register_data[0] / (double)accel->sensivity) * EARTH_GRAVITY;
	accel->data[1] = (accel->register_data[1] / (double)accel->sensivity) * EARTH_GRAVITY;
	accel->data[2] = (accel->register_data[2] / (double)accel->sensivity) * EARTH_GRAVITY;

	double square = sqrt(pow(accel->data[0], 2) + pow(accel->data[1], 2) + pow(accel->data[2], 2));

	accel->data[0] = asin(accel->data[0] / square) * SENSORS_RADS_TO_DPS;
	accel->data[1] = asin(accel->data[1] / square) * SENSORS_RADS_TO_DPS;
	accel->data[2] = 0;
}


/**
 * @brief gyroscope'dan register degerlerini okur
 *
 * @return Basari durumunda '0' Basarisizlik durumunda '1'
 */
PUBLIC uint8_t set_reg_gyro(HI2C i2cPort, HGYRO gyro)
{
    uint8_t buffer[6] = {0};

	gyro->previous_time = gyro->current_time;

	gyro->current_time  = HAL_GetTick();

	i2cPort->readData(MPU6050_GYRO_XOUT_H, buffer, 6);

	if(i2cPort->getError() != I2CBB_ERROR_NONE)
		return FCFAIL;

	gyro->register_data[0] = (int16_t)( buffer[0] << 8 | buffer[1] );
	gyro->register_data[1] = (int16_t)( buffer[2] << 8 | buffer[3] );
	gyro->register_data[2] = (int16_t)( buffer[4] << 8 | buffer[5] );

	gyro->register_data[0] -= gyro->cal_data[0];
	gyro->register_data[1] -= gyro->cal_data[0];
	gyro->register_data[2] -= gyro->cal_data[0];

	return FCSUCCESS;
}

/**
 * @brief aci degerlerini set eder
 */
PUBLIC void set_degree_gyro(HGYRO gyro)
{
	gyro->elapsed_time  = (gyro->current_time - gyro->previous_time) * MILISECOND_TO_SECOND;

	float period = 1 / gyro->elapsed_time;

	gyro->data[0] = (gyro->register_data[0] / (double)gyro->sensivity) / period;
	gyro->data[1] = (gyro->register_data[1] / (double)gyro->sensivity) / period;
	gyro->data[2] = (gyro->register_data[2] / (double)gyro->sensivity) / period;
}
