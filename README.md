# MNPWMLib
A PWM library for Arduino mkr wifi 1010.<br>
Supports generating a Normal PWM (aka single slope, aka Fast PWM) signal on any digital pin D0 - D9.<br>
Based off a 48Mhz clock.>br>
User can optionally provide a callback routine to ve called when PWM duty limit hit.<br>
User can optionally provide a callback routine to be called each time the PWM top value is reached.<br>
User supplies a prescalar, top and duty value.<br>
Library has a function to calculate best prescalar and top values if needed.<br>
