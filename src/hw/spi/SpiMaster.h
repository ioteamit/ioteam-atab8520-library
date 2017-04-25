/* 
* SpiMaster.h
*
* Created: 11/28/2016 10:31:42 PM
* Author: nm
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
