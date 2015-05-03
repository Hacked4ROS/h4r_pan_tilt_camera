/*
 * pwm.h
 *
 *  Created on: Jan 11, 2015
 *      Author: Christian Holl
 */

#ifndef PWM_H_
#define PWM_H_

#include "../config/config.h"


void pwm_init();
void servo0(uint8_t pwm_value);
void servo1(uint8_t pwm_value);

#endif /* PWM_H_ */
