/*
* Gpio.cpp
*
* Created: 11/25/2016 11:31:33 PM
* Author: mfontane
*/


#include "Gpio.h"


void pinMode(uint8_t pin, uint8_t type, port_pin_pull pull)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	/* Configure LEDs as outputs, turn them off */
	if (type == OUTPUT) {
		pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
		port_pin_set_config(pin, &pin_conf);
		port_pin_set_output_level(pin, LOW);
	} else {
		/* Set buttons as inputs */
		pin_conf.direction  = PORT_PIN_DIR_INPUT;
		pin_conf.input_pull = pull;
		port_pin_set_config(pin, &pin_conf);
	}
}

void digitalWrite(uint8_t pin, uint8_t status)
{
	port_pin_set_output_level(pin, status);
}

volatile bool db;
bool digitalRead(uint8_t pin) {
	db = port_pin_get_input_level(pin);
	return db;
}