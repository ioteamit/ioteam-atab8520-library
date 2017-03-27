/* 
* Gpio.h
*
* Created: 11/25/2016 11:31:33 PM
* Author: mfontane
*/


#ifndef __GPIO_H__
#define __GPIO_H__

#include "port.h"
#define OUTPUT PORT_PIN_DIR_OUTPUT
#define INPUT  PORT_PIN_DIR_INPUT

#define LOW 0
#define HIGH 1

void pinMode(uint8_t pin, uint8_t type, port_pin_pull pull=PORT_PIN_PULL_UP);
void digitalWrite(uint8_t pin, uint8_t status);
bool digitalRead(uint8_t pin);

#endif //__GPIO_H__
