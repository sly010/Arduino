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


const static uint8_t A4 = 10;
const static uint8_t A5 = 11;
const static uint8_t A6 = 12;
const static uint8_t A7 = 13;
const static uint8_t A8 = 14;
const static uint8_t A9 = 15;
const static uint8_t A10 = 16;
const static uint8_t A11 = 17;

// heat pins to digital IO
const static uint8_t H0 = A4;
const static uint8_t H1 = A5;
const static uint8_t H2 = A6;
const static uint8_t H3 = A7;
const static uint8_t H4 = A8;
const static uint8_t H5 = A9;
const static uint8_t H6 = A10;
const static uint8_t H7 = A11;

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

// D10  A4   xxx        H0       
// D11  A5   xxx        H1
// D12  A6   xxx        H2
// D13  A7   xxx        H3
// D14  A8   xxx        H4
// D15  A9   xxx        H5
// D16  A10  xxx        H6
// D17  A11  xxx        H7

// PWM0      PB7        OC0A,OC1C   PCINT7/#RTS
// PWM1      PE6                    INT.6/AIN0
// PWM2      PD0        OC0B,       SCL/INT0
// PWM3      PD1        xxxx        SDA/INT1
// PWM4      PD5        xxxx        XCK1/#CTS
// PWM5      PC6        OC3A,#OC4A
// PWM6      PE2                    #HWB
// PWM7      PC7        OC4A        CLK0/ICP3

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

const uint8_t PROGMEM digital_pin_to_port_PGM[30] = {
	PD, // D0 - PD2
	PD,	// D1 - PD3
	PD, // D2 - PD1
	PD,	// D3 - PD0
	PD,	// D4 - PD4
	PC, // D5 - PC6
	PD, // D6 - PD7
	PE, // D7 - PE6
	
	PB, // D8 - PB4
	PB,	// D9 - PB5
	PB, // D10 - PB6
	PB,	// D11 - PB7
	PD, // D12 - PD6
	PC, // D13 - PC7
	
	PB,	// D14 - MISO - PB3
	PB,	// D15 - SCK - PB1
	PB,	// D16 - MOSI - PB2
	PB,	// D17 - SS - PB0
	
	PF,	// D18 - A0 - PF7
	PF, // D19 - A1 - PF6
	PF, // D20 - A2 - PF5
	PF, // D21 - A3 - PF4
	PF, // D22 - A4 - PF1
	PF, // D23 - A5 - PF0
	
	PD, // D24 / D4 - A6 - PD4
	PD, // D25 / D6 - A7 - PD7
	PB, // D26 / D8 - A8 - PB4
	PB, // D27 / D9 - A9 - PB5
	PB, // D28 / D10 - A10 - PB6
	PD, // D29 / D12 - A11 - PD6
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[30] = {
	_BV(2), // D0 - PD2
	_BV(3),	// D1 - PD3
	_BV(1), // D2 - PD1
	_BV(0),	// D3 - PD0
	_BV(4),	// D4 - PD4
	_BV(6), // D5 - PC6
	_BV(7), // D6 - PD7
	_BV(6), // D7 - PE6
	
	_BV(4), // D8 - PB4
	_BV(5),	// D9 - PB5
	_BV(6), // D10 - PB6
	_BV(7),	// D11 - PB7
	_BV(6), // D12 - PD6
	_BV(7), // D13 - PC7
	
	_BV(3),	// D14 - MISO - PB3
	_BV(1),	// D15 - SCK - PB1
	_BV(2),	// D16 - MOSI - PB2
	_BV(0),	// D17 - SS - PB0
	
	_BV(7),	// D18 - A0 - PF7
	_BV(6), // D19 - A1 - PF6
	_BV(5), // D20 - A2 - PF5
	_BV(4), // D21 - A3 - PF4
	_BV(1), // D22 - A4 - PF1
	_BV(0), // D23 - A5 - PF0
	
	_BV(4), // D24 / D4 - A6 - PD4
	_BV(7), // D25 / D6 - A7 - PD7
	_BV(4), // D26 / D8 - A8 - PB4
	_BV(5), // D27 / D9 - A9 - PB5
	_BV(6), // D28 / D10 - A10 - PB6
	_BV(6), // D29 / D12 - A11 - PD6
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
	7,	// A0				PF0					ADC0
	6,	// A1				PF1					ADC1
	5,	// A2				PF4					ADC4
	4,	// A3				PF5					ADC5

	// the followings are heat pins
	7,	// A4				PF0					ADC0
	6,	// A5				PF1					ADC1
	5,	// A6				PF4					ADC4
	4,	// A7				PF5					ADC5
	7,	// A8				PF0					ADC0
	6,	// A9				PF1					ADC1
	5,	// A10				PF4					ADC4
	4,	// A11				PF5					ADC5

};

#endif /* ARDUINO_MAIN */
#endif /* Pins_Arduino_h */
