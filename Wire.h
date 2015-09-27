/*
  Wire.h - a wrapper(+) class for TWI/I2C Master library for the ATtiny on Arduino
  1/21/2011 BroHogan -  brohoganx10 at gmail dot com

  Thanks to 'jkl' for the gcc version of Atmel's USI_TWI_Master code
  http://www.cs.cmu.edu/~dst/ARTSI/Create/PC%20Comm/
  I added Atmel's original Device dependant defines section back into USI_TWI_Master.h
 
 
 NOTE! - It's very important to use pullups on the SDA & SCL lines! More so than with the Wire lib.

	This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or any later version.
  This program is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
  PARTICULAR PURPOSE.  See the GNU General Public License for more details.
*/

#ifndef WIRE_H
#define WIRE_H

#include <inttypes.h>
#include <Arduino.h>
#include <Stream.h>



// User can modify this to change the maximum buffer size so that they can hold more data in a single transfer.
#define WIRE_BUFFER_SIZE 16


// TODO: define Arduino error codes for endTransmission.

// one cannot expect a half-software implementation to go very fast. As a result of test,
// ATTiny needs ~10 cycles to setup loops, so for an 8MHz chip, one have about 30 cycles
// of CPU time left to perform tasks for 100KHz standard speed. 400KHz is a joke. Even at
// 20MHz, one only has 15 cycles left, so, no fast mode guys! Sorry!
// It turns out that interrupts take a lot more time than loops, so the previous interrupt
// design was seriously stupid, and Atmel has a reason to use blocking delays.
#define WIRE_HIGH_TIME 6 // > 4.7 us, considering rise time, make it 6 on the safe side.
#define WIRE_LOW_TIME 5 // > 4 us.


// MCU dependent port&pin defines. Currently supports x5 and x4 tinies.
#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__)
    #define WIRE_DDR			DDRB
    #define WIRE_PORT			PORTB
    #define WIRE_PIN			PINB
    #define WIRE_DDR_SDA		DDB0
    #define WIRE_DDR_SCL		DDB2
    #define WIRE_PORT_SDA		PORTB0
    #define WIRE_PORT_SCL		PORTB2
    #define WIRE_PIN_SDA		PINB0
    #define WIRE_PIN_SCL		PINB2
#endif

#if defined(__AVR_ATtiny24__) | defined(__AVR_ATtiny44__) | defined(__AVR_ATtiny84__)
    #define WIRE_DDR			DDRA
    #define WIRE_PORT			PORTA
    #define WIRE_PIN			PINA
    #define WIRE_DDR_SDA		DDA6
    #define WIRE_DDR_SCL		DDA4
	#define WIRE_PORT_SDA		PORTA6
    #define WIRE_PORT_SCL		PORTA4
    #define WIRE_PIN_SDA		PINA6
    #define WIRE_PIN_SCL		PINA4
#endif


// the defines for ACK and NACK.
#define WIRE_NACK_BIT	0x01


class TinyWire : public Stream
{
public:
 	TinyWire() {
		m_buffer_index = m_current_index = 0;
	}
	/**
	* This is the setup method for I2C master device. Since the master does not have to
	* be aware of anything not initiated by it, there isn't anything done and shouldn't
	* do anything because there is no end() method to denote an enclosing scope. USI is a
	* public interface, and its use should be limited to as small a scope as possible.
	*/
	void begin() {};
	void begin(uint8_t address);
    void beginTransmission(uint8_t slave_addr);
	// two version of write. the third version is defined by Print already. No need to repeat.
	virtual size_t write(uint8_t data);
	virtual size_t write(const uint8_t* buffer, size_t size);
    uint8_t endTransmission(bool stop=true);
    uint8_t requestFrom(uint8_t addr, uint8_t quantity, bool stop=true);
    virtual int available(); 
	virtual int read();
	virtual int peek() {};
	virtual void flush() {};


private:
	uint8_t m_buffer[WIRE_BUFFER_SIZE];
	uint8_t m_buffer_index, m_current_index;
	
	void master_stop();
	bool transfer_byte(uint8_t byte);
	uint8_t read_byte(bool nack);

};

//===================== Definition of inline methods =========================//
inline void TinyWire::beginTransmission(uint8_t slave_addr) {
	// current index is only cleared here. Meaning that multiple calls to endTransmission will not work, preventing errors.
	m_buffer_index = m_current_index = 0;
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
//============================== Private Methods =============================//
inline void TinyWire::master_stop() {
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
	// load the byte into USIDR.
	USIDR = byte;
	// debug note: unlike what one thinks intuitively, USICS pins set to 10 doesn't really shift the data register on rising edge, but on the falling edge of SCL.
	// Need to crunch the document more to understand that. Also, setting USICS to 11 will do weird stuff, where the start condition cannot be correct generated.
	// When the SDA line is pulled low, the SCL line will go low one cycle before that and bounce back to HIGH magically...
	while(!(USISR & (1<<USIOIF))) {
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
	while(!(USISR & (1<<USIOIF))) {
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

// deep black magic part.
// Globally available single instance of Wire.
extern TinyWire Wire;


#endif //WIRE_H

