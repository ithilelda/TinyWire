/*
   TinyWireM.cpp - a wrapper class for TWI/I2C Master library for the ATtiny on Arduino
  1/21/2011 BroHogan -  brohoganx10 at gmail dot com

  **** See TinyWireM.h for Credits and Usage information ****

  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
  PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*/

#include "Wire.h"
#include <Arduino.h>


/**
 * The actual method that does everything. It sets up the USI to be used in TWI master mode,
 * Then it sets up Timer1 registers to interrupt at twice the frequency of transfer rate.
 * Finally, it clears Timer1 and turns on interrupt to start sending data.
 * It only generates the start condition once. There is no repeated start in front of every byte.
 */
uint8_t TinyWire::endTransmission(bool stop) {
	if(m_current_index > m_buffer_index) return 4; // repeatedly call endTransmission without calling beginTransmission.
	// setup the two pins for SCL and SDA as output.
	WIRE_DDR |= (1<<WIRE_DDR_SCL);
	WIRE_DDR |= (1<<WIRE_DDR_SDA);
	// hold high on them to signify a release state.
	WIRE_PORT |= (1<<WIRE_PORT_SCL);
	WIRE_PORT |= (1<<WIRE_PORT_SDA);
	// the data register have to be set to 0xFF too. If one of USIDR OR PORT register is 0, the SDA line will be low.
	USIDR = 0xFF;
	// then we can setup the USICR/USISR registers to enable TWI mode.
    USICR = (0<<USISIE) | (0<<USIOIE) |                           // Disable Interrupts.
            (1<<USIWM1) | (0<<USIWM0) |                           // Set USI in Two-wire mode.
            (1<<USICS1) | (0<<USICS0) | (1<<USICLK);              // Software Strobe for SCL.
	// clear flags in registers. Also resets the counter to 0.
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
	// generate the starting condition. hold SCL line high, and pull SDA line low.
	WIRE_PORT &= ~(1<<WIRE_PORT_SDA);
	// according to spec, at least 4 us should be waited before the first clock cycle generates.
	delayMicroseconds(WIRE_LOW_TIME);
	// Then we pull SCL low, and release the SDA line for data transmission.
	WIRE_PORT &= ~(1<<WIRE_PORT_SCL);
	WIRE_PORT |= (1<<WIRE_PORT_SDA);

	// The main loop. may later put into another routine.
	while(m_current_index <= m_buffer_index) {
		// clear flags in registers. Also resets the counter to 0.
		USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
		// load the next byte into USIDR.
		USIDR = m_buffer[m_current_index++];
		while(USISR & (1<<USIORF)) {
			delayMicroseconds(WIRE_LOW_TIME);
			USICR |= (1<<USITC); // flip clock to high.
			delayMicroseconds(WIRE_HIGH_TIME);
			USICR |= (1<<USITC); // flip clock to low.
		}
		// after the loop, we are at the LOW clock of the 8th bit.
		delayMicroseconds(WIRE_LOW_TIME);
		// we need to release the SDA line. (PORTA4 was already written to 1 previously, so we only need to reset USIDR.)
		USIDR = 0XFF;
		// and make SDA input to read the ACK bit.
		WIRE_DDR &= ~(1<<WIRE_DDR_SDA);
		USICR |= (1<<USITC); // flip clock to high.
		delayMicroseconds(WIRE_HIGH_TIME);
		USICR |= (1<<USITC); // flip clock to low.
		delayMicroseconds(WIRE_LOW_TIME);
		// make SDA output again.
		WIRE_DDR |= (1<<WIRE_DDR_SDA);
		// Now, bit 0 of USIDR is the ACK bit, check it. if NACK is received, we break the loop. If ACK is received, continue our transfer operation.
		m_nack = USIDR & WIRE_NACK_BIT;
		if(m_nack) break;
	}
		
	// regardless of error, we need to issue a STOP condition to release the bus.
	// hold the SDA line low, SCL line high first.
	WIRE_PORT |= (1<<WIRE_PORT_SCL);
	WIRE_PORT &= ~(1<<WIRE_PORT_SDA);
	// a hack here: the spec specifies a minimum delay but no maximum, so we always delay the maximum time at 100KHz, 5us.
	// Arduino guarantees that delay above 3us is quite accurate, so 5us is fine for all CPU frequency.
	delayMicroseconds(WIRE_LOW_TIME);
	// then we release SDA. STOP condition is done.
	WIRE_PORT |= (1<<WIRE_PORT_SDA);
	delayMicroseconds(WIRE_HIGH_TIME);
	// we then need to disable USI.
	USICR = 0;
	// returning status code defined by Arduino depending on the result of operation.
	if(m_nack & m_current_index == 0) return 2;
	else if(m_nack) return 3;
	else return 0;
	
}

uint8_t TinyWire::requestFrom(uint8_t slaveAddr, uint8_t numBytes){ // setup for receiving from slave
  
}

int TinyWire::read(){ // returns the bytes received one at a time
  
}

int TinyWire::available(){ // the bytes available that haven't been read yet
  
}


// Definition of the names in header.
TinyWire Wire = TinyWire();
