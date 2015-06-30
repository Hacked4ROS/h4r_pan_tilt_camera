/*
 * main.c
 *
 *  Created on: Jan 11, 2015
 *      Author: Christian Holl
 */
#include "config/init.h"

int main()
{
	//Init System parts
	init();



	while(1)
	{
		continue;
		for (uint16_t i = 0; i < 10000; ++i);
		servo0(0);
		for (uint16_t i = 0; i < 10000; ++i);
		servo1(0);
		for (uint16_t i = 0; i < 10000; ++i);
		servo0(180);
		for (uint16_t i = 0; i < 10000; ++i);
		servo1(180);


		for (int var = 0; var <= 180; ++var)
		{
			servo0(var);
			servo1(var);
			for (int i = 0; i < 50; ++i);
		}

		for (int var = 180; var >= 0; --var)
		{
			servo0(var);
			servo1(var);
			for (int i = 0; i < 50; ++i);
		}
	}
	return 0;
}
