/* 
* SpiMaster.h
*
* Created: 11/28/2016 10:31:42 PM
* Author: mfontane
*/


#ifndef __SPIMASTER_H__
#define __SPIMASTER_H__

#include <stdint-gcc.h>
#include "spi.h"


class SpiMaster
{
//variables
public:
protected:
private:
//! [dev_inst]
struct spi_module spi_master_instance;
//! [dev_inst]
//! [slave_dev_inst]
struct spi_slave_inst slave;
//! [slave_dev_inst]

//functions
public:
	SpiMaster();
	~SpiMaster();
	void slaveInit(uint8_t slavePin);
	void write(const uint8_t *data, uint16_t len);
	const uint8_t* read(uint8_t *data, uint16_t wLen, uint16_t rLen, uint8_t dummyTx=1);
	
protected:
private:
	SpiMaster( const SpiMaster &c );
	SpiMaster& operator=( const SpiMaster &c );

}; //SpiMaster

#endif //__SPIMASTER_H__
