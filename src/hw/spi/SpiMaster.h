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

#ifndef __SPIMASTER_H__
#define __SPIMASTER_H__

#include <stdint-gcc.h>
#include <SPI.h>

class SpiMaster
{
//variables
public:
protected:
private:

//functions
public:
	SpiMaster();
	~SpiMaster();
	void slaveInit(uint8_t slavePin);
	void write(const uint8_t *data, uint16_t len);
	const uint8_t* read(uint8_t *data, uint16_t wLen, uint16_t rLen, uint8_t dummyTx=1);
        uint8_t* readSPI(uint8_t reg, uint8_t *pReadData, uint16_t wLen, uint16_t rLen, uint8_t dummyTx=1);
	
protected:
private:
	SpiMaster( const SpiMaster &c );
	SpiMaster& operator=( const SpiMaster &c );
	uint8_t ssPin;

}; //SpiMaster

#endif //__SPIMASTER_H__
