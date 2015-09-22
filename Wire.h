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


// Clock speed definitions based on F_CPU declared by Arduino platform. Will treat Timer1 as 8-bit
// regardless of the 16-bit capability on Attinyx4 chips. Since the maximum clock of Attiny can only
// go up to 20MHz, there is no way that the 8-bit counter will overflow even on standard mode. Thus,
// The prescaler is fixed to 1 for maximum precision. Also, the interrupt should trigger on both edges
// of the clock, which means it actually should trigger twice as fast, or count half as many times.
// In I2C spec, you can go slower than the selected mode but not faster (so 90KHz for standard mode is OK, but 110KHz is not).
// Thus, in order to comply to that with 1% accuracy internal oscillator and still not overflow the 8-bit counter at 20MHz,
// I use a 99% clock rate. It doesn't really make a difference at low F_CPU, so you'll need a high clock rate to make a fully compliant
// I2C device.
#ifdef WIRE_FAST_MODE // 400 kHz.
	// The counter needs to count 1.25 on 1MHz to get 400KHz, this is not possible, so we forbid that completely.
	#if defined(F_CPU) && F_CPU == 1000000L
		#error "You cannot use fast mode on a 1MHz CPU."
	#endif
	#define TIMER1_COMPARE F_CPU / 792000L
#else // 100 kHz.
	#define TIMER1_COMPARE F_CPU / 198000L
#endif


// MCU dependent port&pin defines. Currently supports x5 and x4 tinies.
#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__) | \
    defined(__AVR_AT90Tiny26__) | defined(__AVR_ATtiny26__)
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
    #define WIRE_DDR_SDA		DDA5
    #define WIRE_DDR_SCL		DDA4
	#define WIRE_PORT_SDA		PORTA5
    #define WIRE_PORT_SCL		PORTA4
    #define WIRE_PIN_SDA		PINA5
    #define WIRE_PIN_SCL		PINA4
#endif

#define WIRE_BUFFER_SIZE 16

// the defines for the bitfield m_flags.
#define WIRE_RISING_EDGE	0x01			// The 1st bit is the boolean flag to indicate if the clock is a rising edge.
#define WIRE_FINISHED		0x02			// The 2nd bit indicates if the transmission has finished or not.

// defines for state machine.
#define WIRE_STATE_BYTE_START		0x01
#define WIRE_STATE_WRITE			0x02
#define WIRE_STATE_ACK				0x03
#define WIRE_STATE_STRETCH			0x04
#define WIRE_STATE_READ				0x05


class TinyWire : public Stream
{
  private:
	uint8_t m_buffer[WIRE_BUFFER_SIZE];
	uint8_t m_buffer_index, m_current_index, m_nack;
	uint8_t m_flags, m_state;
	
  public:
 	TinyWire() {
		m_buffer_index = m_current_index = 0;
		m_flags = m_state = m_nack = 0;
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
	
	// the special kid.
	void on_tick();
};

// deep black magic part.
// Globally available single instance of Wire.
extern TinyWire Wire;

// typedef that saves typing.
typedef void (TinyWire::*TransferCallback) (void);

// globally available callback function pointer that has the type transfer_callback. (used by Timer1 interrupt handler)
// will move to platform specific files later when I start writing a platform.
extern TransferCallback transfer_callback;

#endif //WIRE_H

