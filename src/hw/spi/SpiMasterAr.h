/* 
* SpiMaster.h
*
* Created: 11/28/2016 10:31:42 PM
* Author: nm
*/


#ifndef __SPIMASTER_AR_H__
#define __SPIMASTER_AR_H__

#include <stdint-gcc.h>
#include <SPI.h>

class SpiMasterAr
{
//variables
public:
protected:
private:

//functions
public:
	SpiMasterAr();
	~SpiMasterAr();
	void slaveInit(uint8_t slavePin);
	void write(const uint8_t *data, uint16_t len);
	const uint8_t* read(uint8_t *data, uint16_t wLen, uint16_t rLen, uint8_t dummyTx=1);
	
protected:
private:
	SpiMasterAr( const SpiMasterAr &c );
	SpiMasterAr& operator=( const SpiMasterAr &c );
	uint8_t ssPin;

}; //SpiMaster

#endif //__SPIMASTER_AR_H__
