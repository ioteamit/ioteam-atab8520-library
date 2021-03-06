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

#include "SpiMaster.h"
//#include "..\dustyWiFoxHw.h"

#define spi_speed  125000

// default constructor
SpiMaster::SpiMaster()
{
	  		
} //SpiMaster

// default destructor
SpiMaster::~SpiMaster()
{
} //~SpiMaster

void SpiMaster::slaveInit(uint8_t slavePin)
{
	this->ssPin=slavePin;
	
	pinMode(slavePin, OUTPUT);
	digitalWrite(slavePin, HIGH);

	// start the SPI library:
	SPI1.begin();
}

void SpiMaster::write(const uint8_t *pData, uint16_t len)
{
	unsigned char* data = (unsigned char *)pData;
	
        Serial.println();
        Serial.print("Write: ");
	SPI1.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
	   
	// take the chip select low to select the device:
	digitalWrite(ssPin, LOW);
	    
	// if you still have another byte to read:
	while (len > 0) {
	  SPI1.transfer(*data);
          Serial.print(" 0x");
          Serial.print(*data,HEX);
          Serial.print(" ");
	  // decrement the number of bytes left to read:
	  // increment the data pointer
	  len--;
	  data++;
	}
        Serial.println();
	// take the chip select high to de-select:
	digitalWrite(ssPin, HIGH);
	    
	SPI1.endTransaction();
}


const uint8_t* SpiMaster::read(uint8_t *pReadData, uint16_t wLen, uint16_t rLen, uint8_t dummyTx)
{
	volatile unsigned char *data = pReadData;
        uint16_t len = rLen;	
    
	SPI1.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
	
	// take the chip select low to select the device:
	digitalWrite(ssPin, LOW);

	SPI1.transfer(*data); // Register Address
        for (int i=0; i< dummyTx; i++) {
	    SPI1.transfer(0x00);
        } 

	// if you still have another byte to read:
	while (len > 0) {
		// shift the first byte left, then get the second byte:
		*data = SPI1.transfer(0x00);
		// decrement the number of bytes left to read:
		len--;
		data++;
	}

	// take the chip select high to de-select:
	digitalWrite(ssPin, HIGH);
	
	SPI1.endTransaction();

	return (const uint8_t*)pReadData;
}

uint8_t* SpiMaster::readSPI(uint8_t reg, uint8_t *pReadData, uint16_t wLen, uint16_t rLen, uint8_t dummyTx)
{
	volatile unsigned char *data = pReadData;
        uint16_t len = rLen;	
	SPI1.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
	
	// take the chip select low to select the device:
	digitalWrite(ssPin, LOW);

	SPI1.transfer(*data); // Register Address

        for (int i=0; i< dummyTx; i++) {
	    SPI1.transfer(0x00);
        } 

	// if you still have another byte to read:
	while (len > 0) {
		// shift the first byte left, then get the second byte:
		*data = SPI1.transfer(0x00);
		// decrement the number of bytes left to read:
		len--;
		data++;
	}

	// take the chip select high to de-select:
	digitalWrite(ssPin, HIGH);
	
	SPI1.endTransaction();
	return (uint8_t*)pReadData;
}
