/*
 * protocol_definition.h
 *
 *  Created on: Jan 15, 2015
 *      Author: Christian Holl
 */

#ifndef PROTOCOL_DEFINITION_H_
#define PROTOCOL_DEFINITION_H_

#define BAUD 9600

typedef enum
{
	CAM_TRANSMIT_TILT,
	CAM_TRANSMIT_PAN,
	CAM_RESET,
}reppy_avr_cmds_t;

typedef union
{
	uint8_t cam_tilt;
	uint8_t cam_pan;
}payload_t;

#define HEADER_ARRAY_INIT 		   { 0xAA, 0x55, 0xA5, 0x5A }
#define CMD_ARRAY_INIT         	   {\
										CAM_TRANSMIT_TILT,\
										CAM_TRANSMIT_PAN,\
										CAM_RESET,\
									};

#define PAYLOAD_LEN_ARRAY_INIT {1,1,0}

#endif /* PROTOCOL_DEFINITION_H_ */
