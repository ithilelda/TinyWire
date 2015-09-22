# TinyWire
Revived project from Rambo to support I2C on ATTiny.

Complete rewrite, not using the Atmel implementation.

Changes:
==============
1. TinyWire class now derives from Stream class. The static object is named 'Wire' to mimic standard Wire library behavior. You can easily port code written for other Arduino boards to Attinys.

2. Supoort ATTinyx4 series (24/44/84).

3. Uses Timer1 to accurately generate timing events, now it should comform better to the standard.

Future plan:
=======
1. Incorporate Slave functionality.

2. Clock stretch implementation.

Installation:
==========
  Rather than installing as a library, I suggest putting it into the hardware folder alongside other functionalities provided by your Attiny platform.
