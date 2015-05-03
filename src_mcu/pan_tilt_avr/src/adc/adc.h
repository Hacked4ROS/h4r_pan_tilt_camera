/*
 * adc.h
 *
 *  Created on: Jan 11, 2015
 *      Author: Christian Holl
 */

#ifndef ADC_H_
#define ADC_H_

#include "../config/config.h"

/**
 * initializes the ADC setup
 */
void adc_init();


/**
 * ADC conversion complete
 */
ISR(ADC_vect);

#endif /* ADC_H_ */
