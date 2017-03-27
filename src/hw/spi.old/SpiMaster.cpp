/*
* SpiMaster.cpp
*
* Created: 11/28/2016 10:31:41 PM
* Author: mfontane
*/


#include "SpiMaster.h"
#include "..\dustyWiFoxHw.h"

// default constructor
SpiMaster::SpiMaster()
{
	//! [config]
	struct spi_config config_spi_master;
	//! [config]
	
	/* Configure, initialize and enable SERCOM SPI module */
	//! [conf_defaults]
	spi_get_config_defaults(&config_spi_master);
	//! [conf_defaults]
	//! [mux_setting]
	config_spi_master.mux_setting = SFX_MASTER_MUX_SETTING;
	//! [mux_setting]
	
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_0;

	config_spi_master.pinmux_pad0 = SFX_MASTER_PINMUX_MISO_PAD0;
	config_spi_master.pinmux_pad1 = SFX_MASTER_PINMUX_PAD1;
	config_spi_master.pinmux_pad2 = SFX_MASTER_PINMUX_MOSI_PAD2;
	config_spi_master.pinmux_pad3 = SFX_MASTER_PINMUX_SCK_PAD3;

	//! [init]
	spi_init(&spi_master_instance, SFX_MASTER_SPI_MODULE, &config_spi_master);
	//! [init]

	//! [enable]
	spi_enable(&spi_master_instance);
	//! [enable]

} //SpiMaster

// default destructor
SpiMaster::~SpiMaster()
{
} //~SpiMaster

void SpiMaster::slaveInit(uint8_t slavePin)
{
	//! [slave_config]
	struct spi_slave_inst_config slave_dev_config;
	//! [slave_config]
	/* Configure and initialize software device instance of peripheral slave */
	//! [slave_conf_defaults]
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	//! [slave_conf_defaults]
	//! [ss_pin]
	slave_dev_config.ss_pin = slavePin;
	//! [ss_pin]
	//! [slave_init]
	spi_attach_slave(&slave, &slave_dev_config);
	//! [slave_init]
}

void SpiMaster::write(const uint8_t *data, uint16_t len)
{
	//! [select_slave]
	spi_select_slave(&spi_master_instance, &slave, true);
	//! [select_slave]
	//! [write]
	spi_write_buffer_wait(&spi_master_instance, data, len);
	//! [write]
	//! [deselect_slave]
	spi_select_slave(&spi_master_instance, &slave, false);
	//! [deselect_slave]
}


const uint8_t* SpiMaster::read(uint8_t *data, uint16_t wLen, uint16_t rLen, uint8_t dummyTx)
{
	// add the eventual dummy in Tx to give time to slave for answer
	uint8_t txLen = wLen+dummyTx; 
	
	//! [select_slave]
	spi_select_slave(&spi_master_instance, &slave, true);
	//! [select_slave]
	//! [write]
	spi_write_buffer_wait(&spi_master_instance, data, txLen);
	spi_read_buffer_wait(&spi_master_instance, &data[txLen], rLen, 0);
	//! [write]
	//! [deselect_slave]
	spi_select_slave(&spi_master_instance, &slave, false);
	//! [deselect_slave]
	
	return &data[txLen];
}
