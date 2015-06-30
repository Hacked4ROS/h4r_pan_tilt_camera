
//void uart_init()
//{
//	//USART Enable 9600 baud, 8 bit, 1 stop bit, no parity,
//	UCSR0A|=(1<<U2X0);
//	UCSR0B|=(1<<RXCIE0)|(1<<TXEN0)|(1<<RXEN0);
//	UCSR0C|=(1<<UCSZ01)|(1<<UCSZ00)|(1<<USBS0);
//	UBRR0=UBRRH_VALUE;
//}

/*
 * uart.c
 *
 *  Created on: Jan 11, 2015
 *      Author: Christian Holl
 */



#include "uart.h"

#include <sersyncproto.h>
#include "../pwm/pwm.h"


void uart_sendByte(uint8_t byte)
{
	//Wait for empty output buffer
	while (!( UCSR0A & (1<<UDRE0)));
	UDR0 = byte;
}

#include <util/setbaud.h>

void uart_init(void)
{
    UBRR0 = UBRR_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   	/* Enable RX and TX */
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); 				/* 8-bit data */
}



ISR(USART_RX_vect)
{
	static const uint8_t cmds[]=CMD_ARRAY_INIT;
	static const uint8_t payload_len[]=PAYLOAD_LEN_ARRAY_INIT;
	static const uint8_t header[]=HEADER_ARRAY_INIT;
	static payload_t payload_buffer;
	INIT_SERSYNCPROTO_DATA(data, cmds, payload_len, header, &payload_buffer);
	uint8_t cur_byte=UDR0;
	if(sersyncproto_rec(&data,cur_byte))
	{
		switch (SERSYNCPROTO_GET_CUR_CMD(data))
		{
			case CAM_TRANSMIT_TILT:
			{
				if(payload_buffer.cam_tilt>=0||payload_buffer.cam_tilt<=180)
				servo0(payload_buffer.cam_tilt);
			}
			break;

			case CAM_TRANSMIT_PAN:
			{
				if(payload_buffer.cam_pan>=0||payload_buffer.cam_pan<=180)
				servo1(payload_buffer.cam_pan);
			}
			break;

			case CAM_RESET:
			{
				servo0(90);
				servo1(90);
			}
			break;
			default:
				break;
		}
	}
}
