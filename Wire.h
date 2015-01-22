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

#ifndef TINYWIRE_H
#define TINYWIRE_H

#include <inttypes.h>
#include "Stream.h"
#define USI_SEND         0              // indicates sending to TWI
#define USI_RCVE         1              // indicates receiving from TWI
#define USI_BUF_SIZE    16              // bytes in message buffer

class TinyWire : public Stream
{
  private:
	static uint8_t USI_Buf[];           // holds I2C send and receive data
	static uint8_t USI_BufIdx;          // current number of bytes in the send buff
	static uint8_t USI_LastRead;        // number of bytes read so far
	static uint8_t USI_BytesAvail;      // number of bytes requested but not read
	
  public:
 	TinyWire();
	void begin();
    void beginTransmission(uint8_t);
    uint8_t endTransmission();
    uint8_t requestFrom(uint8_t, uint8_t);
	virtual size_t write(uint8_t);
    virtual int available(); 
	virtual int read();
	virtual int peek() {};
	virtual void flush() {};
};

extern TinyWire Wire;

#endif //TINYWIRE_H

