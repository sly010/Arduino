/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define ARDUINO_MODEL_USB_PID	0x0034

#define TX_RX_LED_INIT	DDRB |= (1<<0), DDRB |= (1<<0)
#define TXLED0			PORTB |= (1<<0)
#define TXLED1			PORTB &= ~(1<<0)
#define RXLED0			PORTB |= (1<<0)
#define RXLED1			PORTB &= ~(1<<0)

//const static uint8_t SDA = 2;
//const static uint8_t SCL = 3;

// Mapping of analog pins as digital I/O
const static uint8_t A0 = 2;
const static uint8_t A1 = 3;
const static uint8_t A2 = 4;
const static uint8_t A3 = 5;

// reserve 6-9 for SPI
const static uint8_t SS   = 6;
const static uint8_t MOSI = 7;
const static uint8_t MISO = 8;
const static uint8_t SCK  = 9;

// heat pins to digital IO
const static uint8_t H0 = 10;
const static uint8_t H1 = 11;
const static uint8_t H2 = 12;
const static uint8_t H3 = 13;
const static uint8_t H4 = 14;
const static uint8_t H5 = 15;
const static uint8_t H6 = 16;
const static uint8_t H7 = 17;

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA32U4 / HEAT IT

// D0        PD3                 TXD1/INT3
// D1        PD2                 RXD1/INT2

// D2  A0    PF5  ADC5           TCK  
// D3  A1    PF4  ADC4           TMS
// D4  A2    PF1  ADC1
// D5  A3    PF0  ADC0

// D6        PB3	    MISO     PCINT3/PDO
// D7        PB1	    SCK      PCINT1
// D8        PB2	    MOSI     PCINT2/PDI
// D9        PB0	    SS       PCINT0

// D10  H0   PB7                 OC0A/OC1C/PCINT7/#RTS
// D11  H1   PE6                 INT.6/AIN0
// D12  H2   PD0                 OC0B/SCL/INT0
// D13  H3   PD1                 SDA/INT1
// D14  H4   PD5                 XCK1/#CTS
// D15  H5   PC6                 OC3A/#OC4A
// D16  H6   PE2                 #HWB
// D17  H7   PC7                 OC4A/CLK0/ICP3

// A4        ADC9
// A5        ADC8
// A6        ADC10
// A7        ADC11
// A8        ADC12
// A9        ADC13
// A10       ADC6
// A11       ADC7

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[18] = {
	PD, // D0 - PD3
	PD, // D1 - PD2
	
	PF, // D2 - PF5 - A0
	PF, // D3 - PF4 - A1
	PF, // D4 - PF1 - A2
	PF, // D5 - PF0 - A3

	PB, // D6 - PB3 - MISO
	PB, // D7 - PB1 - SCK
	PB, // D8 - PB2 - MOSI
	PB, // D9 - PB0 - SS

	PB, // D10 - PB7
	PE, // D11 - PE6
	PD, // D12 - PD0
	PD, // D13 - PD1
	PD, // D14 - PD5
	PC, // D15 - PC6
	PE, // D16 - PE2
	PC, // D17 - PC7
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[18] = {
	_BV(3), // D0 - PD3
	_BV(2), // D1 - PD2
	
	_BV(5), // D2 - PF5 - A0
	_BV(4), // D3 - PF4 - A1
	_BV(1), // D4 - PF1 - A2
	_BV(0), // D5 - PF0 - A3

	_BV(3), // D6 - PB3 - MISO
	_BV(1), // D7 - PB1 - SCK
	_BV(2), // D8 - PB2 - MOSI
	_BV(0), // D9 - PB0 - SS

	_BV(7), // D18 - PB7
	_BV(6), // D19 - PE6
	_BV(0), // D20 - PD0
	_BV(1), // D21 - PD1
	_BV(5), // D22 - PD5
	_BV(6), // D23 - PC6
	_BV(2), // D24 - PE2
	_BV(7), // D25 - PC7
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[18] = {
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,	
	
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
};

const uint8_t PROGMEM analog_pin_to_channel_PGM[4] = {
	5,	// A0				PF5					ADC5
	4,	// A1				PF4					ADC4
	1,	// A2				PF1					ADC1
	0,	// A3				PF0					ADC0

	// the followings are heat pins
	9,	// A4									ADC9
	8,	// A5									ADC8
	10,	// A6									ADC10
	11,	// A7									ADC11
	12,	// A8									ADC12
	13,	// A9									ADC13
	6,	// A10									ADC6
	7,	// A11									ADC7

};

#endif /* ARDUINO_MAIN */
#endif /* Pins_Arduino_h */
