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
            (0<<USICS1) | (0<<USICS0);                            // Software strobe as counter clock source.
	
	// setup Timer1 registers. (currently only ATTinyx4 platform)
	// disable everything and stops the timer.
	TCCR1A = TCCR1B = TCCR1C = 0;
	// write the target count to OCR1A.
	OCR1A = TIMER1_COMPARE;
	// enable OCR1A interrupt.
	TIMSK1 = (1<<OCIE1A);
	
	// generate the starting condition. hold SCL line high, and pull SDA line low.
	WIRE_PORT &= ~(1<<WIRE_PORT_SDA);
	// according to spec, at least 4 us or 0.6 us should be waited before the first clock cycle generates.
	// since the Timer1 has to count once before the first edge is toggled, this should be satisfied easily.
	// Also, clock stretching is not supported ATM, but the space below is left open for future implementation.
	
	// clear flags in both registers and the object.
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
	m_flags = m_nack = 0;
	// reset the counter register to 0, and start ticking clocks.
	TCNT1H = TCNT1L = 0;
	TCCR1B |= (1<<CS10);
	// the entering state is then WIRE_STATE_BYTE_START.
	m_state = WIRE_STATE_BYTE_START;
	

	// since the Arduino specifies that endTransmission is a blocking process, we block until finished.
	while(!(m_flags & WIRE_FINISHED));
	
	// regardless of error, we need to issue a STOP condition to release the bus. before that, we need to disable USI.
	// this is to prevent the latch on SDA from functioning, so that USIDR does not affect pin value.
	USICR = 0;
	// hold the SDA line low, SCL line high first.
	WIRE_DDR |= (1<<WIRE_DDR_SDA);
	WIRE_PORT |= (1<<WIRE_PORT_SCL);
	WIRE_PORT &= ~(1<<WIRE_PORT_SDA);
	// a hack here: the spec specifies a minimum delay but no maximum, so we always delay the maximum time at 100KHz, 5us.
	// Arduino guarantees that delay above 3us is quite accurate, so 5us is fine for all CPU frequency.
	delayMicroseconds(5);
	// then we pull the SDA line high. STOP condition is done.
	WIRE_PORT |= (1<<WIRE_PORT_SDA);
	// since the Timer1 and USI are already released, we just return status code based on error.
	if(m_nack & m_current_index == 0) return 2;
	else if(m_nack) return 3;
	else return 0;
	
}

// The actual work doer that performs tasks on each interrupt. Implemented as a state machine.
void TinyWire::on_tick() {
	switch(m_state) {
		case WIRE_STATE_BYTE_START:
			// if this tick the clock is going HIGH, we enter the writing state.
			if(m_flags & WIRE_RISING_EDGE) m_state = WIRE_STATE_WRITE;
			// otherwise, we perform setup task on the falling edge tick. This is
			// only performed right after START condition. After each byte is transferred,
			// the setup is done in WIRE_STATE_ACK state.
			else {
				USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
				USIDR = m_buffer[m_current_index++];
			}
			break;
		case WIRE_STATE_WRITE:
			// writing is simple, on every rising edge, advance USIDR.
			if(m_flags & WIRE_RISING_EDGE) USICR |= (1<<USICLK);
			// if on the falling edge the counter is >= 7, it means we've transferred all bits in this byte, and we should enter ACK state.
			else if(USISR & 0x0F >= 7) m_state = WIRE_STATE_ACK;
			break;
		case WIRE_STATE_ACK:
			// if it is the rising edge (the edge when we enter this state), we need to perform setup to allow reading.
			if(m_flags & WIRE_RISING_EDGE) {
				WIRE_DDR &= ~(1<<WIRE_DDR_SDA);
				USIDR = 0xFF;
			}
			else {
				// on the falling edge we advance clock and read in SDA. The trick here: I2C spec requires that the SDA stays stable during the HIGH period of clock,
				// so on the falling edge (which is not trigger yet), the clock is HIGH, and the SDA line's reading is guaranteed to be correct.
				USICR |= (1<<USICLK);
				// In software mode, the SDA reading that's gonna shift into USIDR is sampled the previous instruction cycle, which better guarantees the correctness of SDA reading.
				// now, the LSB in USIDR is the ACK bit returned from the slave.
				m_nack = USIDR & 1;
				// if a NACK is received, or m_current_index reaches m_buffer_index, we stop the Timer1 and raise the WIRE_FINISHED flag.
				if(m_nack || m_current_index > m_buffer_index) {
					TCCR1B = 0;
					m_flags |= WIRE_FINISHED;
				}
				// otherwise an ACK is received, and we continue with the next byte.
				else {
					// the gotcha here: this is at the falling edge of ACK bit, and the next edge is the rising edge for the first bit of next byte.
					// so there is no time for BYTE_START state to setup stuff, we need to setup them here.
					WIRE_DDR |= (1<<WIRE_DDR_SDA);
					USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);
					USIDR = m_buffer[m_current_index++];
					m_state = WIRE_STATE_BYTE_START;
				}
			}
			break;
		case WIRE_STATE_READ:
			break;
			
		default:
			break;
	}
	// regardless of the state, the clock has to change edges every tick.
	USICR |= (1<<USITC);
	m_flags ^= WIRE_RISING_EDGE;
}

uint8_t TinyWire::requestFrom(uint8_t slaveAddr, uint8_t numBytes){ // setup for receiving from slave
  
}

int TinyWire::read(){ // returns the bytes received one at a time
  
}

int TinyWire::available(){ // the bytes available that haven't been read yet
  
}


// Definition of the names in header.
TinyWire Wire = TinyWire();
TransferCallback transfer_callback = NULL;
