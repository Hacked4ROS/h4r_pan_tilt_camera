/*
 * globals.h
 *
 *  Created on: Jan 13, 2015
 *      Author: Christian Holl
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_


#include "../config/config.h"

typedef struct
{
	uint16_t adc_channel_value[8];
	int16_t motor[2];
}global_t;

extern global_t global;


#endif /* GLOBALS_H_ */
