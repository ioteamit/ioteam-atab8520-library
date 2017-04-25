/* 
* Sfx.h
*
* Created: 11/28/2016 11:27:42 PM
* Author: mfontane
*/


#ifndef __SFX_H__
#define __SFX_H__

#include <stdint-gcc.h>
#include "hw/spi/SpiMaster.h"
#include "iot/SigFox/ATAB8520.h"

//NINO
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
	Sfx(uint8_t ssPin, uint8_t resetPin, uint8_t pwronPin, uint8_t eventPin);
	~Sfx();
	void powerOn(bool power);
	uint8_t getEventPin();
	void systemReset(void);
	bool setPortCIOMode(uint8_t pin, uint8_t mode);
	bool portCWrite(uint8_t pin, uint8_t value);
	uint8_t portCRead(uint8_t pin);
	void startHomologation(uint8_t freq[4]);
	const uint8_t* getVersion(void);
	void writeTX(const uint8_t *buffer, uint8_t bufferLen);
	void enableSpecialMode(void);
	const uint8_t* getSigfoxVersion(void);
	void readDeviceStatus(void);
	bool isSystemReady(void);
	bool isFrameReady(void);
	bool isPAIndicationOn(void);
	atmel_error getAtmelStatus(void);
	sigfox_error getSigFoxStatus(void);
	sigfox2_error getSigFox2Status(void);
	void sendSingleBit(void);
	void sendOutOfBand(void);
	void sendFrame(void);
	unsigned long sendMessage(uint8_t* message, int len);
	void sendReceiveFrame(void);
	const uint8_t* getPac(void);
	const uint8_t* readTX(void);
	unsigned long sendReceiveMessage(uint8_t* message, int len, const uint8_t* received);
	void storeSystemConfig(uint8_t eddrc, uint8_t eportc, uint8_t repeat, bool supply3V, bool uplinkOnly, bool EUmode);
	const uint8_t* getID(void);
	supplyTemp readSupplyTemp(void);
	void measureSupplyTemp(void);
	bool testMode(uint8_t mode, uint8_t configuration);
	void sendCW(bool on);
	void firmwareTest(void);
	void storeFrequencies(long freqTX, long freqRX);
	void setTXFreq(long freq);
	void setRXFreq(long freq);
	bool storeCrystalCoeff(uint8_t index, uint8_t data);
	const uint8_t* getCrystalCoeff(void);
	sysConf getSystemConf(void);
	uint8_t getFCCData(void);
	void enableFixedFreq(void);
	void storeChannelConf(void);
	void resetChannelUsage(void);
	void adjustRSSI(uint8_t value);
	bool waitUntilSystemIsReady(unsigned long timeout);
	bool waitUntilEvent(unsigned long timeout);
	void testPowerOnOff(void);
	uint8_t sleepModule(bool sleep);

protected:
private:
	Sfx(){};
	Sfx( const Sfx &c );
	Sfx& operator=( const Sfx &c );
	void setRegister(uint8_t reg);

}; //Sfx

#endif //__SFX_H__
