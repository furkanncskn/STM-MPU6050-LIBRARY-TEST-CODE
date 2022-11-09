/*
 * complementary_filter.cpp
 *
 *  Created on: 24 Mar 2022
 *      Author: Furkan
 */

#include "complementary_filter.h"

FILTER_VAL complementary_filter(HCMPFilter cmpFilter, double teta_g, double teta_a)
{
	cmpFilter->angle = (teta_g + cmpFilter->prev_angle) * cmpFilter->alpha + teta_a * (1 - cmpFilter->alpha);

	cmpFilter->prev_angle = cmpFilter->angle;

	return cmpFilter->angle;
}
