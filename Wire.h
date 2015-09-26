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
#include "Stream.h"


#define WIRE_BUFFER_SIZE 16

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
  private:
	uint8_t m_buffer[WIRE_BUFFER_SIZE];
	uint8_t m_buffer_index, m_current_index, m_nack;
	
  public:
 	TinyWire() {
		m_buffer_index = m_current_index = 0;
		m_nack = 0;
	}
	/**
	* This is the setup method for I2C master device. Since the master does not have to
	* be aware of anything not initiated by it, there isn't anything done and shouldn't
	* do anything because there is no end() method to denote an enclosing scope. USI is a
	* public interface, and its use should be limited to as small a scope as possible.
	*/
	void begin() {};
	// only upload the address to buffer.
    void beginTransmission(uint8_t slave_addr) {
		m_buffer_index = m_current_index = 0; // current index is only cleared here. Meaning that multiple calls to endTransmission will not work, preventing errors.
		m_buffer[m_buffer_index] = slave_addr;
		++m_buffer_index;
	}
	// two version of write. the third version is defined by Print already.
	virtual size_t write(uint8_t data) {
		if (m_buffer_index >= WIRE_BUFFER_SIZE) return 0;
		m_buffer[m_buffer_index] = data;
		++m_buffer_index;
		return 1;
	}
	virtual size_t write(const uint8_t* buffer, size_t size) {
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
    uint8_t endTransmission(bool stop=true);
    uint8_t requestFrom(uint8_t, uint8_t);
    virtual int available(); 
	virtual int read();
	virtual int peek() {};
	virtual void flush() {};
	void begin(uint8_t address);

};

// deep black magic part.
// Globally available single instance of Wire.
extern TinyWire Wire;


#endif //WIRE_H

