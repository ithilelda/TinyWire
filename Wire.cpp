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

//========================= Public Methods ===================================//

//======== Master Write Related =============//

/**
 * The actual method that does everything. It sets up the USI to be used in TWI master mode,
 * Then it sets up Timer1 registers to interrupt at twice the frequency of transfer rate.
 * Finally, it clears Timer1 and turns on interrupt to start sending data.
 * It only generates the start condition once. There is no repeated start in front of every byte.
 */
uint8_t TinyWire::endTransmission(bool stop) {
	if(m_current_index > m_buffer_index) return 4; // repeatedly call endTransmission without calling beginTransmission.
	// setup the two pins for SCL and SDA as output.
	WIRE_DDR |= (1<<WIRE_DDR_SCL) | (1<<WIRE_DDR_SDA);
	// hold high on them to signify a release state.
	WIRE_PORT |= (1<<WIRE_PORT_SCL) | (1<<WIRE_PORT_SDA);
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

	// The main loop. We run until all bytes in buffer is transferred.
	bool nack;
	while(m_current_index <= m_buffer_index) {
		nack = transfer_byte(m_buffer[m_current_index]);
		++m_current_index;
		if(nack) break; // if NACK bit is received, the transmission has failed, we don't need to go further.
	}
	
	// If the stop parameter passed in is true, that means we need to generate a stop signal to release the bus. (see arduino.cc library reference)
	if(stop) master_stop();
	// we also need to release USI, so other protocols can use it.
	USICR = 0;
	// returning status code defined by Arduino depending on the result of operation.
	if(nack & m_current_index == 0) return 2;
	else if(nack) return 3;
	else return 0;
}

//============ Master Read Related =================//
uint8_t TinyWire::requestFrom(uint8_t slave_addr, uint8_t quantity, bool stop){
	if(quantity > WIRE_BUFFER_SIZE) return 0; // we can't handle transfers with more than 16 bytes. There is not enough buffer to hold that.
  	m_buffer_index = quantity - 1; // index starts from 0.
	m_current_index = 0;
	// setup USI.
	WIRE_DDR |= (1<<WIRE_DDR_SCL);
	WIRE_DDR |= (1<<WIRE_DDR_SDA);
	WIRE_PORT |= (1<<WIRE_PORT_SCL);
	WIRE_PORT |= (1<<WIRE_PORT_SDA);
	USIDR = 0xFF;
	USICR = (0<<USISIE) | (0<<USIOIE) |
            (1<<USIWM1) | (0<<USIWM0) |
            (1<<USICS1) | (0<<USICS0) | (1<<USICLK);
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
	// generate start condition.
	WIRE_PORT &= ~(1<<WIRE_PORT_SDA);
	delayMicroseconds(WIRE_LOW_TIME);
	WIRE_PORT &= ~(1<<WIRE_PORT_SCL);
	WIRE_PORT |= (1<<WIRE_PORT_SDA);
	// first we send the address byte.
	bool nack = transfer_byte(slave_addr);
	if(nack) return 0; // no slave answering.
	// then we receive data for quantity - 1 bytes. Reason below.
	while(m_current_index < m_buffer_index) {
		m_buffer[m_current_index] = read_byte(false);
		++m_current_index;
	}
	// in order to save instructions on branch prediction, we write a little more code. 
	// To stop slave from transferring more data, we need to answer the last byte with NACK.
	m_buffer[m_current_index] = read_byte(true);
	// reset m_current_index so that we can read by using available() and read() according to Arduino style.
	// finally, generate stop condition based on the stop boolean.
	if(stop) master_stop();
	// and releases USI.
	USICR = 0;
	return m_current_index;
}
int TinyWire::read(){
	uint8_t tmp = m_buffer[m_current_index];
	++m_current_index;
	return tmp;
}
int TinyWire::available(){
	return m_buffer_index - m_current_index;
}

// Definition of the names in header.
TinyWire Wire = TinyWire();
