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
inline void TinyWire::beginTransmission(uint8_t slave_addr) {
	m_buffer_index = m_current_index = 0; // current index is only cleared here. Meaning that multiple calls to endTransmission will not work, preventing errors.
	m_buffer[m_buffer_index] = slave_addr;
	++m_buffer_index;
}
inline size_t TinyWire::write(uint8_t data) {
	if (m_buffer_index >= WIRE_BUFFER_SIZE) return 0;
	m_buffer[m_buffer_index] = data;
	++m_buffer_index;
	return 1;
}
inline size_t TinyWire::write(const uint8_t* buffer, size_t size) {
	if (m_buffer_index >= WIRE_BUFFER_SIZE) return 0;
	size_t left = WIRE_BUFFER_SIZE - m_buffer_index;
	if (size > left) {
		// there is not enough space.
		memcpy(m_buffer + m_buffer_index, buffer, left);
		m_buffer_index += left;
		return left;
	}
	else {
		// there is enough space.
		memcpy(m_buffer + m_buffer_index, buffer, size);
		m_buffer_index += size;
		return size;
	}
}
/**
 * The actual method that does everything. It sets up the USI to be used in TWI master mode,
 * Then it sets up Timer1 registers to interrupt at twice the frequency of transfer rate.
 * Finally, it clears Timer1 and turns on interrupt to start sending data.
 * It only generates the start condition once. There is no repeated start in front of every byte.
 */
inline uint8_t TinyWire::endTransmission(bool stop) {
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

	// The main loop. We run until all bytes in buffer is transferred.
	bool nack;
	while(m_current_index <= m_buffer_index) {
		nack = transfer_byte(m_buffer[m_current_index]);
		++m_current_index;
		if(nack) break; // if NACK bit is received, the transmission has failed, we don't need to go further.
	}
	
	// If the stop parameter passed in is true, that means we need to generate a stop signal to release the bus. (see arduino.cc library reference)
	if(stop) stop();
	// we also need to release USI, so other protocols can use it.
	USICR = 0;
	// returning status code defined by Arduino depending on the result of operation.
	if(nack & m_current_index == 0) return 2;
	else if(nack) return 3;
	else return 0;
}

//============ Master Read Related =================//
inline uint8_t TinyWire::requestFrom(uint8_t slave_addr, uint8_t quantity, bool stop){
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
		m_buffer[m_current_index] = receive_byte(false);
		++m_current_index;
	}
	// in order to save instructions on branch prediction, we write a little more code. 
	// To stop slave from transferring more data, we need to answer the last byte with NACK.
	m_buffer[m_current_index] = receive_byte(true);
	// reset m_current_index so that we can read by using available() and read() according to Arduino style.
	// finally, generate stop condition based on the stop boolean.
	if(stop) stop();
	// and releases USI.
	USICR = 0;
	return m_current_index;
}
inline int TinyWire::read(){
	uint8_t tmp = m_buffer[m_current_index];
	++m_current_index;
	return tmp;
}
inline int TinyWire::available(){
	return m_buffer_index - m_current_index;
}

//============================== Private Methods =============================//
inline void TinyWire::stop() {
	// hold the SDA line low, SCL line high first.
	WIRE_PORT |= (1<<WIRE_PORT_SCL);
	WIRE_PORT &= ~(1<<WIRE_PORT_SDA);
	// a hack here: the spec specifies a minimum delay but no maximum, so we always delay the maximum time at 100KHz, 5us.
	// Arduino guarantees that delay above 3us is quite accurate, so 5us is fine for all CPU frequency.
	delayMicroseconds(WIRE_LOW_TIME);
	// then we release SDA. STOP condition is done.
	WIRE_PORT |= (1<<WIRE_PORT_SDA);
	delayMicroseconds(WIRE_HIGH_TIME);
}

// the routine to transfer a single byte. No state modified.
inline bool TinyWire::transfer_byte(uint8_t byte) {
	// clear flags in registers. Also resets the counter to 0.
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
	// load the next byte into USIDR.
	USIDR = byte;
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
	// make SDA output again.
	WIRE_DDR |= (1<<WIRE_DDR_SDA);
	// Now, return bit 0 of the USIDR, which is the ACK/NACK bit.
	return USIDR & WIRE_NACK_BIT;
}

// the routine to receive a single byte. No state modified.
inline uint8_t TinyWire::read_byte(bool nack) {
	// clear flags in registers. Also resets the counter to 0.
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
	while(USISR & (1<<USIORF)) {
		delayMicroseconds(WIRE_LOW_TIME);
		USICR |= (1<<USITC); // flip clock to high.
		delayMicroseconds(WIRE_HIGH_TIME);
		USICR |= (1<<USITC); // flip clock to low.
	}
	// after the loop, we are at the LOW clock of the 8th bit. First read out USIDR.
	uint8_t data = USIDR;
	// then prepare to generate ACK bit.
	// first, make SDA output and release it (set HIGH).
	WIRE_DDR |= (1<<WIRE_DDR_SDA);
	WIRE_PORT |= (1<<WIRE_PORT_SDA);
	// depending on the request of the caller, we either generate ACK (nack = false), or NACK (nack = true).
	if(nack) USIDR = 0xFF;
	else USIDR = 0x00;
	// finally we can flip the clocks.
	delayMicroseconds(WIRE_LOW_TIME);
	USICR |= (1<<USITC); // flip clock to high.
	delayMicroseconds(WIRE_HIGH_TIME);
	USICR |= (1<<USITC); // flip clock to low.
	// make SDA input again.
	WIRE_DDR &= ~(1<<WIRE_DDR_SDA);
	// regardless of ACK condition, we return the data.
	return data;
}

// Definition of the names in header.
TinyWire Wire = TinyWire();
