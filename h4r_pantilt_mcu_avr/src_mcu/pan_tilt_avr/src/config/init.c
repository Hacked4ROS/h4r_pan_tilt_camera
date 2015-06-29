/*
 * init.c
 *
 *  Created on: Jan 11, 2015
 *      Author: Christian Holl
 */

#include "init.h"

void init()
{
	uart_init();
	adc_init();
	pwm_init();
	sei();
}
