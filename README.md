# MNPWMLib
A PWM library for Arduino mkr wifi 1010.<br>
Supports generating a Normal PWM (aka single slope, aka Fast PWM) signal on any digital pin D0 - D9.<br>
Based off a 48Mhz clock.<br>
User can optionally provide a callback routine to be called when PWM duty limit hit.<br>
User can optionally provide a callback routine to be called each time the PWM top value is reached.<br>
User supplies a prescalar, top and duty value.<br>
Library has a function to calculate best prescalar and top values if needed.<br>

Note: this code uses attachInterruptParam. There is a known issue in the arduino 1.8.13 release for SAMD where this function is in the standard include libraries but not included in the actual libraries that are linked. This will cause an unresolved linkage error. <br>
This has a fix but is not yet released, see:<br>
https://github.com/arduino/ArduinoCore-samd/pull/667<br>
https://github.com/arduino/ArduinoCore-samd/issues/624<br>

To fix this either wait for the next release and check it is resolved or pro tem replace the Winterrupts.cpp file with the fixed version.

The resolved version can be found here:<br>
https://github.com/arduino/ArduinoCore-samd/blob/9a67d2d1ffa281328ba6965fe6d9f6245e18df5c/cores/arduino/WInterrupts.c<br>

Assuming you have installed arduino for your own user, this should replace the version installed as part of 1.8.13 on your local machine at <br>
C:\Users\\[your username]\AppData\Local\Arduino15\packages\arduino\hardware\samd\1.8.13\cores\arduino
