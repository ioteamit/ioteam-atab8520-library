/*
* Sfx.cpp
*
* Created: 11/28/2016 11:27:42 PM
* Author: mfontane 
*/


#include "..\Sfx.h"
#include "..\hw\gpio\Gpio.h"
#include "..\delay\delay.h"
//#include "ATAB8520.h"



// default constructor
Sfx::Sfx(uint8_t ssPin, uint8_t resetPin, uint8_t pwronPin, uint8_t eventPin)
{
	this->ssPin		= ssPin;
	this->resetPin	= resetPin;
	this->pwronPin	= pwronPin;
	this->eventPin	= eventPin;
	
	pinMode(resetPin, OUTPUT);
	pinMode(pwronPin, OUTPUT);
	pinMode(eventPin, INPUT, PORT_PIN_PULL_NONE);
	delay_ms(1000); //delay_us(50); really not need


	/* reset cycle
	* as per Figure 2-2. Power-up Sequence
	* http://www.atmel.com/Images/Atmel-9372-Smart-RF-ATA8520_Datasheet.pdf
	*/
	// move in OFF Mode
	digitalWrite(pwronPin, LOW);
	digitalWrite(resetPin, LOW);
	delay_ms(2000); //delay_us(20);
	// activate the on cycle
	digitalWrite(pwronPin, HIGH);
	delay_ms(2000); //delay_ms(20)
	// everything is ready remove reset
	digitalWrite(resetPin, HIGH);
	
	comunication.slaveInit(ssPin);
} //Sfx


// default destructor
Sfx::~Sfx()
{
} //~Sfx

uint8_t freqTx[]={0x33, 0xBE, 0x9C, 0xD0};
uint8_t freqRx[]={0x33, 0xD3, 0xE6, 0x08};
uint8_t atmelVer[]={0x06, 0x00, 0x00, 0x00};
uint8_t data3[]={0x0A, 0x00, 0x00, 0x00, 0x00};

void Sfx::powerOn(bool power)
{
	if(power) {
		digitalWrite(pwronPin, HIGH);
	} else {
		digitalWrite(pwronPin, LOW);		
	}
}

uint8_t Sfx::getEventPin()
{
	return digitalRead(this->eventPin);
}

void Sfx::startHomologation(uint8_t freq[])
{	
	// first set the frequency
	setRegister(SET_TX_FREQ);
	setDataToWrite(freqTx, sizeof(freqTx));
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	
	setRegister(SET_RX_FREQ);
	setDataToWrite(freqRx, sizeof(freqRx));
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	
	// than set the CW Mode
	setRegister(SEND_CW);
	setDataToWrite(freq, sizeof(freq));
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

uint8_t Sfx::sleepModule(bool sleep)
{
	if (sleep) {
		setRegister(POWER_OFF);
		comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	}
	
	return 0x00;
}

void Sfx::systemReset(void)
{
	setRegister(SYSTEM_RESET);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

//Usage: setPortCIOMode(4, OUTPUT) to put in OUTPUT mode PortC pin 4
bool Sfx::setPortCIOMode(uint8_t pin, uint8_t mode)
{
	if(pin<1 || pin>5 || (mode!=INPUT && mode!=OUTPUT)) {
		return false;
	}
	
	if(mode == INPUT) {
		this->portCPin &= ~(1 << pin); //RESET
	} else if(mode == OUTPUT) {
		this->portCPin |= 1 << pin; //SET
	}
	
	setRegister(SET_PC_IO_MODE);
	setDataToWrite(&this->portCPin, sizeof(this->portCPin));
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	
	return true;
}

bool Sfx::portCWrite(uint8_t pin, uint8_t value)
{
	if(pin<1 || pin>5 || (value!=LOW && value!=HIGH)) {
		return false;
	}

	if(value == HIGH) {
		this->portCValue &= ~(1 << pin); //RESET
	} else if(value == LOW) {
		this->portCValue |= 1 << pin; //SET
	}

	setRegister(PC_WRITE);
	setDataToWrite(&this->portCValue, sizeof(this->portCValue));
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	
	return true;
}

uint8_t Sfx::portCRead(uint8_t pin)
{
	if(pin<0 || pin>5) {
		return 255;
	}
	
	setRegister(PC_READ);
	uint8_t retValue = *comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, PC_READ_RET_LEN);
	uint8_t bit = (retValue >> pin) & 1;
	
	return bit;
}

const uint8_t* Sfx:: getVersion(void){
	setRegister(ATMEL_VERSION);
	return comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, ATMEL_VERSION_RET_LEN);
}

void Sfx::writeTX(const uint8_t *buffer, uint8_t bufferLen)
{
	//Max limit 12 bytes
	uint8_t lBuffer[12+1]; //First byte holds frame length
	uint8_t lBufferLen;
	
	if(bufferLen>12) {
		lBufferLen=12;
	} else {
		lBufferLen=bufferLen;
	}
	
	lBuffer[0]=lBufferLen;
	memcpy(lBuffer+1, buffer, lBufferLen);
	//End Max Limit
	
	setRegister(WRITE_TX);
	setDataToWrite(lBuffer, lBufferLen+1); //+ length byte
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::enableSpecialMode(void)
{
	setRegister(ENABLE_SPECIAL_MODE);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

const uint8_t* Sfx:: getSigfoxVersion(void){
	setRegister(SIGFOX_VERSION);
	return comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, SIGFOX_VERSION_RET_LEN);
}

void Sfx::readDeviceStatus(void)
{
	setRegister(DEVICE_STATUS);
	this->deviceStatus = comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, DEVICE_STATUS_RET_LEN);
}

bool Sfx::isSystemReady(void)
{
	if(((this->deviceStatus[1] >> 6) & 1) == 1)
	{
		return true;
		} else {
		return false;
	}
}

bool Sfx::isFrameReady(void)
{
	if(((this->deviceStatus[1] >> 5) & 1) == 1)
	{
		return true;
		} else {
		return false;
	}
}

bool Sfx::isPAIndicationOn(void)
{
	if(((this->deviceStatus[1] >> 0) & 1) == 1)
	{
		return true;
		} else {
		return false;
	}
}

atmel_error Sfx::getAtmelStatus(void)
{	
	return (atmel_error)((this->deviceStatus[1] >> 1) & 0x0F);
}

sigfox_error Sfx::getSigFoxStatus(void)
{
	switch(this->deviceStatus[2])
	{
		case 0x0:
			return SIGFOX_NO_ERROR;
		break;
		case 0x30:
			return SIGFOX_DATA_LEN_GT12;
		break;
		case 0x3E:
			return SIGFOX_TIMEOUT_MESSAGE;
		break;
		case 0x4E:
			return SIGFOX_TIMEOUT_BIT;
		break;
		default:
			return SIGFOX_INTERNAL_ERROR;
		break;		
	}
}

sigfox2_error Sfx::getSigFox2Status(void)
{
	switch(this->deviceStatus[3])
	{
		case 0x0:
			return SIGFOX2_NO_ERROR;
		break;
		case 0x10:
			return SIGFOX2_INITIALIZATION_ERROR;
		break;
		case 0x18:
			return SIGFOX2_SEND_ERROR;
		break;
		case 0x40:
			return SIGFOX2_FREQUENCY_ERROR;
		break;
		case 0x68:
			return SIGFOX2_DATA_TIMEOUT_ERROR;
		break;
		default:
			return SIGFOX2_UNKNOWN_ERROR;
		break;
	}
}

void Sfx::sendSingleBit(void)
{
	setRegister(SEND_SINGLE_BIT);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::sendOutOfBand(void)
{
	setRegister(SEND_OUT_OF_BAND);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::sendFrame(void)
{
	setRegister(SEND_FRAME);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

unsigned long Sfx::sendMessage(uint8_t* message, int len) {
	if(!waitUntilSystemIsReady(10000)) {
		return false;
	}
	
	writeTX(message, len);
	sendFrame();
	
	readDeviceStatus();
	atmel_error ae=getAtmelStatus();
	sigfox_error se=getSigFoxStatus();
	sigfox2_error se2=getSigFox2Status();
	
	if(ae==0 && se==0 && se2==0) {
		return 0;
	} else {
		return (ae || se*16 || se2*16*256);
	}
}

void Sfx::sendReceiveFrame(void)
{
	setRegister(SEND_RECEIVE_FRAME);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

const uint8_t* Sfx::readTX(void)
{
	setRegister(READ_TX); //CHECK LEN (it can be 9)
	return comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, READ_TX_RET_LEN);
}

unsigned long Sfx::sendReceiveMessage(uint8_t* message, int len, const uint8_t* received)
{	
	if(!waitUntilSystemIsReady(10000)) {
		return 98;
	}
	
	writeTX(message, len);
	sendReceiveFrame();
	
	if(!waitUntilEvent(60000)) {
		return 99;
	}
	
	received=readTX();

	atmel_error ae=getAtmelStatus();
	sigfox_error se=getSigFoxStatus();
	sigfox2_error se2=getSigFox2Status();
	
	if(ae==0 && se==0 && se2==0) {
		return 0;
	} else {
		return (ae || se*16 || se2*16*256);
	}
}

const uint8_t* Sfx::getPac(void)
{
	setRegister(PAC_ID);
	return comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, PAC_ID_RET_LEN);
}

//Store configuration data in EEPROM
//eddrc = portC data direction (0=input; 1=output) - pinC0=0 always, pinC6 and pinC7 don't exist
//eportc = portC data value - pinC6 and pinC7 don't exist
//repeat = number of frames to be send for the SPI command Send/Receive Frame - 0x00 = 1 frame, 0x02 = 3 frames
//supply3V => true=3v, false=5v
//uplinkOnly => true=uplink only, false=downlink/uplink
//EUmode => true=EU, false=US
void Sfx::storeSystemConfig(uint8_t eddrc, uint8_t eportc, uint8_t repeat, bool supply3V, bool uplinkOnly, bool EUmode)
{
	uint8_t dataToSend[4];
	
	dataToSend[0]=eddrc;
	dataToSend[1]=eportc;
	if(repeat>0x02) {
		dataToSend[2]=0x02;
	} else {
		dataToSend[2]=repeat;
	}
	dataToSend[3]=0x31; //Default values
	if(supply3V) {
		dataToSend[3] |= 0x08;
	}
	if(uplinkOnly) {
		dataToSend[3] |= 0x04;
	}
	if(EUmode) {
		dataToSend[3] |= 0x02;
	}
	
	setRegister(STORE_SYSTEM_CONFIG);
	setDataToWrite(dataToSend, 4);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

const uint8_t* Sfx::getID(void)
{
	setRegister(GET_ID);
	return comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, GET_ID_RET_LEN);
}

supplyTemp Sfx::readSupplyTemp(void)
{
	supplyTemp retVal;
	const uint8_t* readValues;
	
	setRegister(READ_SUPPLY_TEMP);
	readValues=comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, READ_SUPPLY_TEMP_RET_LEN);
	
	retVal.mVoltIdle=readValues[1]*256+readValues[0];
	retVal.mVoltActive=readValues[3]*256+readValues[2];
	uint8_t sPart=readValues[5] >> 7;  //sign part
	if(sPart==0) {
		retVal.temp = (readValues[5]*256+readValues[4])/10;
	} else {
		retVal.temp = -1*((readValues[5]+128)*256+readValues[4])/10;
	}
	
	return retVal;
}

void Sfx::measureSupplyTemp(void)
{
	setRegister(MEASURE_SUPPLY_TEMP);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

bool Sfx::testMode(uint8_t mode, uint8_t configuration)
{
	if(mode>4) {
		return false;
	}
	setRegister(TEST_MODE);
	setDataToWrite(&mode, 1);
	setDataToWrite(&configuration, 1);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	return true;
}

void Sfx::sendCW(bool on)
{
	uint8_t mode;
	
	if(on) {
		mode=0x11;
	} else {
		mode=0x00;
	}
	
	setRegister(SEND_CW);
	setDataToWrite(&mode, 1);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::firmwareTest(void)
{
	uint8_t data[] = {0x06, 0x09, 0xFF, 0xFF};
		
	setRegister(FIRMWARE_TEST);
	setDataToWrite(data, 4);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::storeFrequencies(long freqTX, long freqRX)
{
	uint8_t data;
	
	setRegister(STORE_FREQUENCIES);
	data = freqTX & 0xFF;
	setDataToWrite(&data, 1);
	data = (freqTX >> 8) & 0xFF;
	setDataToWrite(&data, 1);
	data = (freqTX >> 16) & 0xFF;
	setDataToWrite(&data, 1);
	data = (freqTX >> 24) & 0xFF;
	setDataToWrite(&data, 1);
	data = freqRX & 0xFF;
	setDataToWrite(&data, 1);
	data = (freqRX >> 8) & 0xFF;
	setDataToWrite(&data, 1);
	data = (freqRX >> 16) & 0xFF;
	setDataToWrite(&data, 1);
	data = (freqRX >> 24) & 0xFF;
	setDataToWrite(&data, 1);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::setTXFreq(long freq)
{
	uint8_t f[4];
	memcpy(f,&freq,sizeof(freq));
	setRegister(SET_TX_FREQ);
	setDataToWrite(f, sizeof(freq));
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::setRXFreq(long freq)
{
	uint8_t f[4];
	memcpy(f,&freq,sizeof(freq));
	setRegister(SET_RX_FREQ);
	setDataToWrite(f, sizeof(freq));
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

bool Sfx::storeCrystalCoeff(uint8_t index, uint8_t data)
{
    if(index>22) {
		return false;
	}
	setRegister(STORE_CRYSTAL_COEFF);
	setDataToWrite(&index, 1);
	setDataToWrite(&data, 1);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	return true;
}

const uint8_t* Sfx::getCrystalCoeff(void)
{
	const uint8_t* readValues;

	setRegister(READ_CRYSTAL_COEFF);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	
	setRegister(GET_CONFIG_BUFFER);
	readValues=comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, READ_CRYSTAL_COEFF_RET_LEN);
		
	return readValues;
}

sysConf Sfx::getSystemConf(void)
{
	sysConf sc;
	const uint8_t* readValues;
	
	setRegister(READ_SYS_CONF);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);

	setRegister(GET_CONFIG_BUFFER);
	readValues=comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, READ_SYS_CONF_RET_LEN);

	sc.freqRX=readValues[0]+readValues[1]*256+readValues[2]*256*256+readValues[3]*256*256*256;
	sc.freqTX=readValues[4]+readValues[5]*256+readValues[6]*256*256+readValues[7]*256*256*256;	
	sc.repeat=readValues[8];
	sc.sConf=readValues[9];
	
	return sc;
}

uint8_t Sfx::getFCCData(void)
{
	uint8_t readValue;

	setRegister(READ_FCC_DATA);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
	
	setRegister(GET_CONFIG_BUFFER);
	readValue=*comunication.read(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen, READ_FCC_DATA_RET_LEN);
	
	return readValue;
}

void Sfx::enableFixedFreq(void)
{
	setRegister(ENABLE_FIXED_FREQ);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::storeChannelConf(void)
{
	uint8_t chConf[]={0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
		
	setRegister(STORE_CHANNEL_CONF);
	setDataToWrite(chConf, STORE_CHANNEL_CONF_RET_LEN);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::resetChannelUsage(void)
{
	setRegister(RESET_CHANNEL_USAGE);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

void Sfx::adjustRSSI(uint8_t value)
{
	setRegister(ADJUST_RSSI);
	setDataToWrite(&value, 1);
	comunication.write(sfxMsg.msg.payloadTxRx, sfxMsg.msgLen);
}

bool Sfx::waitUntilSystemIsReady(unsigned long timeout) //timeout in milliseconds
{
	bool ready=false;
	unsigned long counter=0;
	
	while(!ready && counter*100<timeout) {  //timeout check
	  readDeviceStatus();
	  ready=isSystemReady();
	  counter++;
	  delay_ms(100);
	}
	
	if(ready) {
	  return true;	
	} else {
	  return false;
	}
}

bool Sfx::waitUntilEvent(unsigned long timeout) //timeout in milliseconds
{
	uint8_t eventValue=1; //Line event is on when its value is 0
	unsigned long counter=0;
	
	while(eventValue==1 && counter*100<timeout) {  //timeout check
		//readDeviceStatus();
		eventValue=getEventPin();
		counter++;
		delay_ms(100);
	}
	
	if(eventValue==0) {
		return true;
		} else {
		return false;
	}
}

void Sfx::testPowerOnOff(void){
	digitalWrite(pwronPin, LOW);
	digitalWrite(resetPin, LOW);
	sleepModule(true);
	delay_ms(5000); //delay_us(20);
	// activate the on cycle
	digitalWrite(pwronPin, HIGH);
	//delay_ms(2000); //delay_ms(20)
	// everything is ready remove reset
	digitalWrite(resetPin, HIGH);
}



void Sfx::setRegister(uint8_t reg)
{
	memset(sfxMsg.msg.payloadTxRx, 0 , sizeof(sfxMsg.msg)); // clean the old msg
	sfxMsg.msg.address = reg;
	sfxMsg.msgLen = 1;
}

void Sfx::setDataToWrite(const uint8_t *buffer, uint8_t bufferLen)
{
	memcpy(&sfxMsg.msg.payloadTxRx[sfxMsg.msgLen], buffer, bufferLen); // skip the address
	sfxMsg.msgLen += bufferLen;
}

