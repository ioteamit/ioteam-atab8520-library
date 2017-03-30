/*
* SpiMaster.cpp
*
* Created: 02/14/2017 21:40:41 PM
* Author: nm
*/


#include "SpiMaster.h"
#include "..\dustyWiFoxHw.h"

// default constructor
SpiMaster::SpiMaster()
{
	// start the SPI library:
	SPI.begin();
	  		
} //SpiMaster

// default destructor
SpiMaster::~SpiMaster()
{
} //~SpiMaster

void SpiMaster::slaveInit(uint8_t slavePin)
{
	this->ssPin=slavePin;
	
	pinMode(slavePin, OUTPUT);
}

void SpiMaster::write(const uint8_t *pData, uint16_t len)
{
	unsigned char* data = pData;
	
	SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
	   
	// take the chip select low to select the device:
	digitalWrite(ssPin, LOW);
	    
	// if you still have another byte to read:
	while (len > 0) {
	  SPI.transfer(*data);
	  // decrement the number of bytes left to read:
	  // increment the data pointer
	  len--;
	  data++;
	}
	// take the chip select high to de-select:
	digitalWrite(ssPin, HIGH);
	    
	SPI.endTransaction();
}


const uint8_t* SpiMaster::read(uint8_t *pReadData, uint16_t wLen, uint16_t rLen, uint8_t dummyTx)
{
	volatile unsigned char *data = pReadData;
	
	SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
	
	// take the chip select low to select the device:
	digitalWrite(ssPin, LOW);

	// if you still have another byte to read:
	while (rLen > 0) {
		// shift the first byte left, then get the second byte:
		*data = SPI.transfer(0x00);
		// decrement the number of bytes left to read:
		rLen--;
		data++;
	}

	// take the chip select high to de-select:
	digitalWrite(ssPin, HIGH);
	
	SPI.endTransaction();
	
	return &data[0];
}
