/* This file is subject to the MIT license */

/*-----------------------------------------------------*\
|  USI I2C Slave Driver                                 |
|                                                       |
| This library provides a robust, interrupt-driven I2C  |
| slave implementation built on the ATTiny Universal    |
| Serial Interface (USI) hardware.  Slave operation is  |
| implemented as a register bank, where each 'register' |
| is a pointer to an 8-bit variable in the main code.   |
| This was chosen to make I2C integration transparent   |
| to the mainline code and making I2C reads simple.     |
| This library also works well with the Linux I2C-Tools |
| utilities i2cdetect, i2cget, i2cset, and i2cdump.     |
|                                                       |
| Adam Honse (GitHub: CalcProgrammer1) - 7/29/2012      |
|            -calcprogrammer1@gmail.com                 |
| Tobias Schramm (GitHub: TobleMiner) - 3/20/2019       |
|            -tobleminer@gmail.com                      |
\*-----------------------------------------------------*/
#ifndef USI_I2C_SLAVE_H
#define USI_I2C_SLAVE_H

#include <avr/io.h>
#include <avr/interrupt.h>

#include "time.h"

//Microcontroller Dependent Definitions
#if defined (__AVR_ATtiny24__) | \
	defined (__AVR_ATtiny44__) | \
	defined (__AVR_ATtiny84__)
	#define DDR_USI			DDRA
	#define PORT_USI		PORTA
	#define PIN_USI			PINA
	#define PORT_USI_SDA	PA6
	#define PORT_USI_SCL	PA4
	#define PIN_USI_SDA		PINA6
	#define PIN_USI_SCL		PINA4
#endif

#if defined(__AVR_ATtiny85__)
	#define DDR_USI			DDRB
	#define PORT_USI		PORTB
	#define PIN_USI			PINB
	#define PORT_USI_SDA	PB0
	#define PORT_USI_SCL	PB2
	#define PIN_USI_SDA		PINB0
	#define PIN_USI_SCL		PINB2
#endif

#if defined(__AVR_AT90Tiny2313__) | \
	defined(__AVR_ATtiny2313__)
    #define DDR_USI             DDRB
    #define PORT_USI            PORTB
    #define PIN_USI             PINB
    #define PORT_USI_SDA        PB5
    #define PORT_USI_SCL        PB7
    #define PIN_USI_SDA         PINB5
    #define PIN_USI_SCL         PINB7
#endif

struct UCI_ISC_Reg;

struct UCI_ISC_Reg {
	struct {
		uint8_t changed:1;
	} attr;
	uint8_t data;
};

//USI I2C Initialize
//  address - If slave, this parameter is the slave address
void USI_I2C_Init(char address, struct UCI_ISC_Reg** regs, uint8_t num_regs);
//void USI_I2C_Set_Timeout(struct timeval_t* timeout);
uint8_t USI_I2C_Busy();

#endif