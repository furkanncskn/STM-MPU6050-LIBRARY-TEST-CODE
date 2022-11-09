/*
 * complementary_filter.h
 *
 *  Created on: Mar 24, 2022
 *      Author: Furkan
 */

#ifndef LIBRARY_FILTER_COMPLEMENTARY_FILTER_H_
#define LIBRARY_FILTER_COMPLEMENTARY_FILTER_H_

#include "MPU6050/inc/fc_mpu6050.h"

typedef double FILTER_VAL;

typedef struct {

	double alpha;
	double angle;
	double prev_angle;

}CMPFilter, *HCMPFilter;

FILTER_VAL complementary_filter(HCMPFilter cmpFilter, double teta_g, double teta_a);

#endif /* LIBRARY_FILTER_COMPLEMENTARY_FILTER_H_ */
