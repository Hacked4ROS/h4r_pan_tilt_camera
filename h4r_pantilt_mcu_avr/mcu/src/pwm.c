/*
 * pwm.c
 *
 *  Created on: Jan 11, 2015
 *      Author: Christian Holl
 */

#include "pwm.h"
#include <util/delay.h>
#define SERVO_FACT ((uint16_t) (servo_max-servo_min)*100/180)
#define SERVO_VALUE(DEG) SERVO_FACT*DEG/100+servo_min
void pwm_init()
{
	//Timer 1 reset
	TCCR1A = 0;
	TCCR1B = 0;

	//TOP Value of the counter
	ICR1 = 5000;

	OCR1A=ICR1;
	OCR1B=ICR1;

	//Fast PWM
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);

	//PRESCALER /8
	TCCR1B |= (1 << CS11)|( 1 << CS10);

	//Set Output Pins
	DDRB |= _BV(1) | _BV(2);

/*
 * 	Set OC1A/OC1B on Compare Match when up-
 *  counting. Clear OC1A/OC1B on Compare Match
 *  when downcounting.
 */
	TCCR1A |= (0x2 <<  6) |  (0x2 <<  4);


	//RESET Position to center
	OCR1A=SERVO_VALUE(90);
	_delay_ms(2000);
	OCR1B=SERVO_VALUE(90);
	_delay_ms(2000);
}

void servo0(uint8_t pwm_value)
{
	OCR1A = SERVO_VALUE(pwm_value); // 2ms pulse to left motor on PB1
}

void servo1(uint8_t pwm_value)
{
	OCR1B = SERVO_VALUE(pwm_value); // 2ms pulse to left motor on PB1
}
