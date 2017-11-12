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



#ifndef __SFX_H__
#define __SFX_H__

#include <Arduino.h>
#include <stdint-gcc.h>
#include "hw/spi/SpiMaster.h"
#include "iot/SigFox/ATAB8520.h"

#define PORT_PIN_PULL_NONE 0
#define delay_ms(X) delay(X)

struct supplyTemp
{
	int mVoltIdle;
	int mVoltActive;
	int temp;
};

struct sysConf
{
	unsigned long freqTX;
	unsigned long freqRX;
	uint8_t repeat;
	uint8_t sConf;
};

class Sfx//: public IotModule
{
//variables
public:
protected:
private:
	SpiMaster comunication;
	uint8_t   ssPin;
	uint8_t   resetPin;
	uint8_t   pwronPin;
	uint8_t   eventPin;
	uint8_t   portCPin;
	uint8_t   portCValue;
	sfxMsgS   sfxMsg;

void setDataToWrite(const uint8_t *buffer, uint8_t bufferLen);
//functions
public:
	uint8_t   deviceStatus[4];

        /*
        * Object constructor (default)
        *
        */
	Sfx(){};

        /*
        * Object constructor
        *
        * <ssPin>: slave pin
        * <resetPin>: reset pin 
        * <pwronPin>: power on pin 
        * <eventPin>: event pin
	* (check schema for pinout)
        * 
        * Response: none
        */
	Sfx(uint8_t ssPin, uint8_t resetPin, uint8_t pwronPin, uint8_t eventPin);

        /*
        * Object destructor
        *
        */
	~Sfx();

        /*
        * Initialize system (default, no input) 
        *
        * This method stores the default system configuration parameters,
	* initialize the communication engine and reset the module
        *
        * Response: none
        */
        void begin();

        /*
        * Set the RX/TX USA frequencies
        *
        * Response: none
        */
        void setEUFreq();

        /*
        * Set the RX/TX EU frequencies
        *
        * Response: none
        */
	void setUSFreq();

        /*
        * Set powerOn pin LOW/HIGH
        *
        * <power>: if true powerOn=HIGH, if false powerOn=LOW 
        * 
        * Response: none
        */
        void powerOn(bool power);

        /*
        * Get event pin status
        *
        * Response: event pin status
        */
	uint8_t getEventPin();

        /*
        * System reset (code 0x01) 
        *
	* This command uses the system internal WDT to do a complete hardware reset of the Atmel ATA8520E.
	* Resetting the device takes approximately 12ms (EU), 31ms (US). Afterwards the system restarts and
	* generates an event on the EVENT signal after this time. This event will be cleared with the “Get Status”
	* SPI command (0x0A).
        * 
        * Response: none
        */
	void systemReset(void);

        /*
        * Set C I/O port mode (code 0x02)
        *
	* The I/O lines of port C can be used as additional I/O lines for an application. The port C I/O Init command
	* defines the internal data direction register of output port PORTC (DDRC). Pin PC0 is used as NRESET
	* signal and will always be an input pin, i.e., bit 0 will be written as 0 to be an input pin.
	*
	* <pin>: pin number (1-5 are valid pins)
	* <mode>: INPUT or OUTPUT 
        *
        * Response: true if ok 
        */
	bool setPortCIOMode(uint8_t pin, uint8_t mode);

        /*
        * Write on I/O port C (code 0x03)
        *
	* The I/O write command writes directly to the output port register PORTC to set the I/O pins. Pin PC0 is
	* used as NRESET signal and will always be an input pin with enabled pull-up, i.e., bit 0 will be written as 1
	* to enable the internal pull-up resistor.
        *
	* <pin>: pin number (1-5 are valid pins)
	* <value>: HIGH or LOW 
        *
        * Response: true if ok
        */
	bool portCWrite(uint8_t pin, uint8_t value);

        /*
        * Read I/O port C (code 0x04)
        *
	* The I/O read command reads the status of the I/O pins directly from the input port register PINC. Pin PC0
	* is used as NRESET signal and will always be read as 1.
        *
	* <pin>: pin number (1-5 are valid pins)
        *
        * Response: pin value (HIGH (1) or LOW (0))
        */
	uint8_t portCRead(uint8_t pin);

        /*
        * (Obsolete)
        * Start homologation (TO BE DELETED)
        * See setEUFreq e setUSFreq
        */
	void startHomologation(uint8_t freq[4]);

        /*
        * Get Atmel version (code 0x06)
        *
	* The Atmel version command reads the version information including a major and a minor version
	* number.
        *
        * Response: Atmel module version
        */
	const uint8_t* getVersion(void);

        /*
        * Write the TX transmission buffer (code 0x07)
	*
	* The write TX buffer command fills the TX buffer to be sent with the next SIGFOX ATA8520E data frame
	* with payload data of up to 12 bytes. The buffer can hold any number of bytes ranging from 0 to 12 bytes
	* and are not buffered, i.e., a new SPI command will override the previous data.
	*
	* <buffer>: pointer to message to send 
	* <input>: message length 
	*
        * Response: none
        */
	void writeTX(const uint8_t *buffer, uint8_t bufferLen);

        /*
        * Enable special mode (code 0x08)
	*
	* This command will only be used during testing of the system and not during regular operation in a
	* SIGFOX network. This commands enables the execution of the following SPI command: Firmware tests 0x18
	* <input>: none
	*
        * Response: none
        */
	void enableSpecialMode(void);

        /*
        * Get SigFox version (code 0x09)
	*
	* The SIGFOX version reads the SIGFOX library version information as a text string with N = 11
	* characters.
	*
        * Response: SigFox version
        */
	const uint8_t* getSigfoxVersion(void);

        /*
        * Read device status (code 0x0A)
	*
	* The get status command reads the internal status of the device. Issuing this command clears the systems
	* event line (PB6) and the status bytes. The event line is set to low when:
	* a. System is ready after power-up or reset
	* b. Finishes the transmit/receive operation
	* c. Finishes a temperature and supply measurement
	* d. Finishes the EEPROM write operation.
	* e. Test mode is finished.
	* The following status information is read after the event line is activated, i.e., polling using the Get Status
	* command is not necessary:
	* Hardware SSM status (internal only)
	* Atmel status:
	* Bit6: System ready to operate (system ready event)
	* Bit5: Frame sent (frame ready event)
	* Bit4 to Bit1: Error code
	* – 0000: no error
	* – 0001: command error / not supported
	* – 0010: generic error
	* – 0011: frequency error
	* – 0100: usage error
	* – 0101: opening error
	* – 0110: closing error
	* – 0111: send error
	* Bit0: PA on/off indication
	* SIGFOX status:
	* 0x00: no error
	* 0x30: TX data length > 12 byte
	* 0x3E: Time-out for downlink message
	* 0x4E: Time-out for bit downlink
	* All other codes: Only for internal
	* SIGFOX status2:
	* 0x00: no error
	* 0x10: initialization error
	* 0x18: error during send
	* 0x40: error in RF frequency
	* 0x68: error during wait for data frame
	* The SSM status is used for internal testing only. The SIGFOX status/status2 information may also
	* generate other error codes which are used for internal only. The Atmel status information can be used for
	* detection of issues with the application, i.e., bit6 is set after initialization of the device (reset or power-on)
	* and bit5 is set after a telegram has been sent.
	*
	* The status is read (used) by:
	* - isSystemReady
	* - isFrameReady
	* - isPAIndicationOn
	* - getAtmelStatus
	* - getSigFoxStatus
	* - getSigFox2Status
	*
        * Response: none
        */
	void readDeviceStatus(void);

        /*
        * Test if the system is ready (Must be used after readDeviceStatus)
	*
        * Response: True is the system is ready
        */
	bool isSystemReady(void);

        /*
        * Test if the frame is ready (Must be used after readDeviceStatus)
	*
        * Response: True if the frame is ready
        */
	bool isFrameReady(void);

        /*
        * Test if the PA indication is on (Must be used after readDeviceStatus)
	*
        * Response: true if the PA indication is on
        */
	bool isPAIndicationOn(void);

        /*
        * Get Atmel status (Must be used after readDeviceStatus)
	*
        * Response: Atmel status (see readDeviceStatus)
        */
	atmel_error getAtmelStatus(void);

        /*
        * Get SigFox status (Must be used after readDeviceStatus)
	*
        * Response: SigFox status (see readDeviceStatus)
        */
	sigfox_error getSigFoxStatus(void);

        /*
        * Get SigFox2 status (Must be used after readDeviceStatus)
	*
        * Response: SigFox2 status (see readDeviceStatus)
        */
	sigfox2_error getSigFox2Status(void);

        /*
        * Send single bit (code 0x0B)
	*
	* This command sends a data bit (0=0x00/1=0x01) within a SIGFOX RF frame as specified by SIGFOX.
	* An event on the EVENT signal is generated when finished. This command will only be used during testing
	* of the system and not during regular operation in a SIGFOX network.
	*
        * Response: none
        */
	void sendSingleBit(void);

        /*
        * Send Out of Band (code 0xC)
	*
	* This command triggers the out-of-band data transmission (as defined by SIGFOX ). It will generate an
	* event on the EVENT signal when finished. This command will only be used during testing of the system
	* and not during regular operation in a SIGFOX network.
	*
        * Response: none
        */
	void sendOutOfBand(void);

        /*
        * Send frame (code 0x0D)
	*
	* The send frame command triggers the start of a frame transmit process. The payload data has to be
	* written into the TX buffer before using the write TX buffer command. The transmit operation will take
	* ~7 seconds in EU mode and ~2 seconds in US mode and will generate an event on the EVENT signal
	* when finished.
	*
        * Response: none
        */
	void sendFrame(void);

        /*
        * Send Message
	*
	* Send a message and wait until an event is received
	*
	* <message>: Pointer to the message to send
	* <len>: message length
	*
        * Response: atmel_error | sigfox_error*16 | sigfox2_error*16*256
        */
	unsigned long sendMessage(uint8_t* message, int len);

        /*
        * Send receive frame (code 0x0E)
	*
	* The send/receive frame command triggers the start of a frame transmit process followed by a receive
	* process. The payload data has to be written into the TX buffer before using the write TX buffer command.
	* The transmit and receive operation will take up to 50 seconds and will generate an event on the EVENT
	* signal when finished. The received data bytes can be read with the SPI command (0x10).
	*
        * Response: none
        */
	void sendReceiveFrame(void);

        /*
        * Get PAC (code 0xF)
	*
	* The get PAC command will read the 16 byte PAC information which is used for the device registration
	* process at the SIGFOX backend. Only the 8 lower bytes (0) .. (7) are used.
        * Response: PAC
        */
	const uint8_t* getPac(void);

        /*
        * Read transmitted packet (code 0x10)
	*
	* This command triggers the read out of the received data packet. The packet length is always 8 bytes.
	*
        * Response: the received data packet
        */
	const uint8_t* readTX(void);

        /*
        * Send and receive message
	*
	* <message>: Pointer to the message to send
	* <len>: message length
	* <received>: message received
	*
        * Response: atmel_error | sigfox_error*16 | sigfox2_error*16*256
        */
	unsigned long sendReceiveMessage(uint8_t* message, int len, const uint8_t* received);

        /*
        * Store system configuration (code 0x11)
	*
	* The Store System Configuration command writes the configuration data for the port C and the system
	* configuration into the internal EEPROM. This changes will be applied by performing a system reset. An
	* event on the EVENT signal is generated when finished. EDDRC register defines the data direction for the
	* port C pins (0: input, 1: output). EPORTC register defines the output level for an output pin and enables a
	* pull-up resistor for input pins when set. SysConf is used to configure the supply voltage and the up-/
	* downlink operation. The parameter repeat defines the
	* number of frames to be send
	* for the SPI command Send/Receive Frame (0x0E). Possible values for the
	* parameter repeat are
	* 0x00: send 1 frame
	* 0x01: send 2 frame
	* 0x02: send 3 frame (default)
	*
	* <eddrc>: data direction for the port C pins (0: in, 1: out)
	* <eportc>: register output level
	* <repeat>: frame repetition (max 3) 
	* <supply3V>: true if the power supply is 3V, false = 5v
	* <uplinkOnly>: true=uplink only, false=downlink/uplink
	* <EUmode>: true=EU, false=US
	*
        * Response: none
        */
	void storeSystemConfig(uint8_t eddrc, uint8_t eportc, uint8_t repeat, bool supply3V, bool uplinkOnly, bool EUmode);

        /*
        * Get ID (code 0x12)
	*
	* The get ID command will read the 4 byte ID information which is used for the device registration process
	* at the SIGFOX backend.
	*
        * Response: ID
        */
	const uint8_t* getID(void);

        /*
        * Read supply temperature (code 0x13)
	*
	* This command triggers the read out of the measured supply voltage in idle and active mode and the
	* device temperature. To trigger a measurement the SPI command (0x14) has to be used. The return
	* voltage level is in mV and the temperature value has to be calculated as T = TM/10 in °C. The voltage
	* values are of type 16 bit unsigned integer (with high and low byte) while the temperature is a signed
	* value.
	*
        * Response: supply temperature
        */
	supplyTemp readSupplyTemp(void);

        /*
        * Measure power supply and temperature (code 0x14)
	*
	* This command triggers the measurement of the supply voltages and the temperature value. An event on
	* the EVENT signal is triggered when finished which is cleared by reading the status with command 0x0A.
	* Using this command will update the crystal calibration before any send command, i.e., it is recommended
	* to adapt to changed ambient temperatures.
	*
        * Response: none
        */
	void measureSupplyTemp(void);

        /*
        * Test Mode (code 0x15)
	*
	* This command triggers the uplink test procedure defined by SIGFOX . An event on the EVENT signal is
	* generated when finished. This command will only be used during testing of the system and not during
	* regular operation in a SIGFOX network.
	* The command parameter are:
	* TestMode: Test modes as defined by SIGFOX
	* Configuration: configuration data for test modes as defined by SIGFOX
	*
	* <mode>: valid modes 0-4
	* <configuration>: configuration data
	*
        * Response: true if the method worked fine
        */
	bool testMode(uint8_t mode, uint8_t configuration);

        /*
        * Send continuous carrier (code 0x17)
	*
	* This command triggers the transmission of a continuous carrier on the programmed RF frequency as
	* defined by SIGFOX . This command will only be used during testing of the system and not during regular
	* operation in a SIGFOX network.
	*
	* <on>: 0x11=on, 0x00=off
	*
        * Response: none
        */
	void sendCW(bool on);

        /*
        * Firmware Test (code 0x18)
	*
	* This command selects the firmware internal RX test mode. An event on the EVENT signal is generated
	* when finished. This command will only be used during testing of the system and not during regular
	* operation in a SIGFOX network.
	*
        * Response: none
        */
	void firmwareTest(void);

        /*
        * Store Frequencies (code 0x1A)
	*
	* <freqTX>: TX frequency in Hz
	* <freqRX>: RX frequency in Hz
	*
        * Response: none
        */
	void storeFrequencies(long freqTX, long freqRX);

        /*
        * Set Tx Frequency (code 0x1B)
	*
	* Set TX center frequency temporarily for testing purposes. This settings are lost after reset or when
	* switching the device off. The frequency value is an unsigned 32-bit integer within the range
	* [868.000.000Hz to 868.600.000Hz] and default value 868.130.000Hz for EU. The range for US is
	* [902.000.000Hz to 906.000.000Hz] with default 902.200.000Hz. This command will only be used during
	* testing of the system and not during regular operation in a SIGFOX network.
	*
	* <freq>: Tx frequency
	*
        * Response: none
        */
	void setTXFreq(long freq);

        /*
        * Set Rx Frequency (code 0x1C)
	*
	* Set RX center frequency temporarily for testing purposes. This settings are lost after reset or when
	* switching the device off. The frequency value is an unsigned 32-bit integer within the range
	* [869.400.000Hz to 869.650.000Hz] and default value 869.525.000Hz for EU. The range for US is
	* [902.000.000Hz to 906.000.000Hz] with default 905.200.000Hz. This command will only be used during
	* testing of the system and not during regular operation in a SIGFOX network.
	*
	* <freq>: Rx requency
	*
        * Response: none
        */
	void setRXFreq(long freq);

        /*
        * Store Crystal Coefficiens (code 0x1D)
	*
	* This command stores a crystal coefficient for temperature compensation at position INDEX (range
	* 0 to 22). The INDEX is related to a specific temperature value, i.e., index position 0 is for –48°C, index 1
	* for –40°C, index 2 for –32°C and so forth until index 22 for +128°C. The data value has to be in ppm and
	* is interpreted as a signed value. The final table is composed with a step size of 8°C, starting at –48°C and
	* ending at +128°C. The command will issue an event when finished.
	*
	* <index>: index position for specific temperature value
	* <data>: value in ppm
	*
        * Response: true if method worked fine
        */
	bool storeCrystalCoeff(uint8_t index, uint8_t data);

        /*
        * Get crystal coefficient table (code 0x1E)
	*
	* This command triggers the read operation of the crystal coefficient table into a buffer area for the
	* temperature range of –32°C to +88°C in steps of 8°C (16 coefficients) for verification purposes. The buffer
	* read itself is then performed with command 0x20.
	*
        * Response: pointer to crystal coefficient table
        */
	const uint8_t* getCrystalCoeff(void);

        /*
        * Get system configuration (code 0x1F)
	*
	* This command triggers the read operation of the center frequencies for up- and downlink in Hz and the
	* system configuration setting as used in command 0x11. The buffer read itself is then performed with
	* command 0x20.
	* <input>:
	*
        * Response: system configuration structure
        */
	sysConf getSystemConf(void);

        /*
        * Get FCC data (code 0x20)
	*
	* This command reads the current FCC channel configuration stored in the EEPROM after issuing the
	* trigger command 0x25. The buffer includes 1 FCC data byte with
	* Bit 7...4: =
	* 1 if the current FCC Macro Channel is not the default SIGFOX channel
	* 0 otherwise
	* Bit 3...0: #
	* of free Micro Channel inside the current FCC Macro Channel
	*
        * Response: FCC data
        */
	uint8_t getFCCData(void);

        /*
        * Enable Fixed Frequency (code 0x21)
	*
	* This command will toggle between frequency hopping and fixed frequency for testing purposes. After
	* applying a reset the frequency hopping mode is enabled per default. This command will only be used
	* during testing of the system and not during regular operation in a SIGFOX network.
	*
        * Response: none
        */
	void enableFixedFreq(void);

        /*
        * Store channel configuration (code 0x22)
	*
	* This command stores the channel configuration for the US mode operation in EEPROM. The following
	* values have to be used for FCC compliance:
	* MC1[0..3]: 0xFF, 0x01, 0x00, 0x00
	* MC2[0..3]: 0x00, 0x00, 0x00, 0x00
	* MC3[0..3]: 0x00, 0x00, 0x00, 0x00
	* DC[0..1]: 0x01, 0x00
	*
        * Response: none
        */
	void storeChannelConf(void);

        /*
        * Reset channel usage (code 0x23)
	*
	* This command will reset the channel configuration of the US mode. It has to be applied before using any
	* send or send/receive command (in US mode only). In addition it has to be ensured in the application
	* software to use this command with a minimum delay of 20 seconds between consecutive calls to comply
	* with FCC regulations.
	*
        * Response: none
        */
	void resetChannelUsage(void);

        /*
        * Adjust RSSI (code 0x24)
	*
	* This command will store a value which is automatically added to the measured RSSI level. This value is
	* derived from the gain or loss of the external circuitry including the antenna. This corrected RSSI value will
	* be used during frame sending to the SIGFOX network and is calculated as (value is of type signed 8-bit
	* data):
	* RSSIsystem = RSSImeasured + Value
	*
	* <value>: value to add to RSSImeasured
	*
        * Response: none
        */
	void adjustRSSI(uint8_t value);

        /*
        * Wait until system is ready
	*
	* It uses the isSystemReady method to determine if the system is ready.
	* 
	* <timeout>: max time to wait in milliseconds
	*
        * Response: true if the system is ready
        */
	bool waitUntilSystemIsReady(unsigned long timeout);

        /*
        * Wait until event
	*
	* It reads the event pin to check for an incoming event.
	*
	* <timeout>: max time to wait in milliseconds
	*
        * Response: true if and event is triggered
        */
	bool waitUntilEvent(unsigned long timeout);

        /*
        * Test power
	*
	* Shut down the system for 5 seconds the switch it on again.
	*
        * Response: none
        */
	void testPowerOnOff(void);

        /*
        * Put the module in sleep mode (code 0x05)
	*
	* <sleep>: true to put the module in sleep mode
	*
        * Response: 0x00
        */
	uint8_t sleepModule(bool sleep);

protected:
private:
	Sfx( const Sfx &c );
	Sfx& operator=( const Sfx &c );
	void setRegister(uint8_t reg);

}; //Sfx

extern Sfx SigFoxObj;

#endif //__SFX_H__
