/*
 * uart.h
 *
 *  Created on: Jan 11, 2015
 *      Author: Christian Holl
 */

#ifndef UART_H_
#define UART_H_

#include "../config/config.h"

/**
 * Init uart interface
 */
void uart_init();

/**
 * Send byte over uart
 */
void uart_sendByte(uint8_t byte);


#endif /* UART_H_ */
