# TinyWire
Revived project from Rambo to support I2C on ATTiny.

Not modifying much. Just maintenance updates to support more devices and conforms to the newest Wire API.

Changes:
1. TinyWire class now derives from Stream class. The static object is named 'Wire' to mimic standard Wire library behavior.
2. Supoort ATTinyx4 series (24/44/84).

Future plan:
1. Incorporate Slave functionality.
2. Look deeper into the code to fine tune for each MCUs, and watch out for conformity issues with Official Arduino Wire.
