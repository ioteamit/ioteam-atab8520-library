
/*
 * delay.c
 *
 * Created: 11/25/2016 11:47:25 PM
 *  Author: mfontane
 */ 
#include <stdint-gcc.h>

/** Tick Counter united by ms */
static volatile uint32_t _ulTickCount=0 ;

uint32_t millis( void )
{
	// todo: ensure no interrupts
	return _ulTickCount ;
}

void SysTick_DefaultHandler(void)
{
	// Increment tick count each ms
	_ulTickCount++;
	//tickReset();
}