/* SmartEverything ATA8520E Library
 * Copyright (C) 2017 by IOTEAM
 *
 * This file is part of the SmartEverything Arduino RN2483 Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SmartEverything RN2483 Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *  Design:      sw@ioteam.it
 */


#ifndef ATAB8520_H_
#define ATAB8520_H_
#include "stdint.h"
/*
CMD					Index	    Write Data			  Read Data
System reset		0x01	None						None
I/O Init			0x02	DDRC register setting		None
I/O Write			0x03	PORTC register setting		None
I/O Read			0x04	None						PINC register setting
OFF mode			0x05	None						None
Atmel version		0x06	None						Major / minor
Write TX buffer		0x07	Data written to TX buffer	None
Test mode (<V1.0)	0x08	Frame/channel				None
Reserved (?V1.0)				-						  -
SIGFOX™ version		0x09	None						Version L-H
Get status			0x0A	None						SSM / Atmel® FW / SIGFOX library
Send bit			0x0B(1) Bit (0/1)					None
Reserved			0x0C		-								-
Send frame			0x0D	None						None
Reserved			0x0E		-								-
Get PAC				0x0F	None						PAC[0], PAC[1] …. PAC[15]
Reserved			0x10		-								-
Write Sys Conf		0x11(1) DDRC, PORTC, SysConf		None
Get ID				0x12	None						ID[3] … ID[0]
Read sup/temp		0x13(1) None						Supply idle / supply active / temperature
Start measurement	0x14(1) None						None
TX test mode		0x15(1) Frame/channel				None
Reserved			0x16		-								-
Send CW				0x17(1) On/Off						None
Reserved			0x18		-								-
Reserved			0x19		-								-
Reserved			0x1A		-								-
Set TX frequency	0x1B(1) TX frequency				None
Reserved			0x1C		-								-
Note: 1. These commands are available in device with Atmel Version V ? 1.0
*/

#define HIGH					1
#define LOW						0

#define SYSTEM_RESET			0x01

#define SET_PC_IO_MODE			0x02

#define PC_WRITE				0x03

#define PC_READ 				0x04
#define PC_READ_RET_LEN 		0x01

#define ATMEL_VERSION			0x06
#define ATMEL_VERSION_RET_LEN	0x02

#define WRITE_TX				0x07

#define ENABLE_SPECIAL_MODE		0x08

#define SIGFOX_VERSION			0x09
#define SIGFOX_VERSION_RET_LEN	0x0B

#define DEVICE_STATUS			0x0A
#define DEVICE_STATUS_RET_LEN	0x04

#define SEND_SINGLE_BIT			0x0B

#define SEND_OUT_OF_BAND		0x0C

#define SEND_FRAME				0x0D

#define SEND_RECEIVE_FRAME		0x0E

#define PAC_ID					0x0F
#define PAC_ID_RET_LEN			0x10

#define READ_TX					0x10
#define READ_TX_RET_LEN			0x08

#define STORE_SYSTEM_CONFIG		0x11

#define GET_ID					0x12
#define GET_ID_RET_LEN			0x04

#define READ_SUPPLY_TEMP		0x13
#define READ_SUPPLY_TEMP_RET_LEN 0x06

#define MEASURE_SUPPLY_TEMP		0x14

#define TEST_MODE				0x15

#define SEND_CW					0x17

#define FIRMWARE_TEST			0x18

#define STORE_FREQUENCIES		0x1A

#define SET_TX_FREQ				0x01B

#define SET_RX_FREQ				0x01C

#define STORE_CRYSTAL_COEFF		0x1D

#define READ_CRYSTAL_COEFF		0x1E
#define READ_CRYSTAL_COEFF_RET_LEN 0x0B

#define READ_SYS_CONF			0x1F
#define READ_SYS_CONF_RET_LEN	0x0A

#define GET_CONFIG_BUFFER		0x20

#define ENABLE_FIXED_FREQ		0x21

#define STORE_CHANNEL_CONF		0x22
#define STORE_CHANNEL_CONF_RET_LEN	0x0E

#define RESET_CHANNEL_USAGE		0x23

#define ADJUST_RSSI				0x24

#define READ_FCC_DATA			0x25
#define READ_FCC_DATA_RET_LEN	0x01


#define POWER_OFF				0x05

#define MAX_ANSWER 17 // including the dummmy tx for the read command
#pragma pack(1)
typedef union {
	uint8_t address;
	uint8_t payloadTxRx[MAX_ANSWER+1]; // consider the Address
}sfxMsgU;

typedef struct  {
	sfxMsgU msg;
	uint8_t msgLen;
}sfxMsgS;

enum atmel_error 
{
	ATMEL_NO_ERROR,
	ATMEL_COMMAND_ERROR,
	ATMEL_GENERIC_ERROR,
	ATMEL_FREQUENCY_ERROR,
	ATMEL_USAGE_ERROR,
	ATMEL_OPENING_ERROR,
	ATMEL_CLOSING_ERROR,
	ATLMEL_SEND_ERROR
};

enum sigfox_error
{
	SIGFOX_NO_ERROR,
	SIGFOX_DATA_LEN_GT12,
	SIGFOX_TIMEOUT_MESSAGE,
	SIGFOX_TIMEOUT_BIT,
	SIGFOX_INTERNAL_ERROR
};

enum sigfox2_error
{
	SIGFOX2_NO_ERROR,
	SIGFOX2_INITIALIZATION_ERROR,
	SIGFOX2_SEND_ERROR,
	SIGFOX2_FREQUENCY_ERROR,
	SIGFOX2_DATA_TIMEOUT_ERROR,
	SIGFOX2_UNKNOWN_ERROR
};

#endif /* ATAB8520_H_ */
