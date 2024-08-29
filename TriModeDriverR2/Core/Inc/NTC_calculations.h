/*
 * Thermistor.h
 *
 *  Created on: Jul 23, 2024
 *      Author: Kingfisher
 */

#ifndef INC_NTC_CALCULATIONS_H_
#define INC_NTC_CALCULATIONS_H_

float thermistorResistance(uint16_t ntc_filt_reading);

float thermistorTemperature(float R);

#endif /* INC_NTC_CALCULATIONS_H_ */
