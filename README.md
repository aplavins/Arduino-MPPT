# Arduino-MPPT
Arduino based Maximum Power Point Tracking (MPPT) solar charge controller code

Author: Adam Plavinskis
        aplavins@gmail.com

This code is for an arduino UNO based Solar MPPT charge controller.
It is based of work done by Julian Ilett (256.co.uk), Debasish Dutta/deba168,
and Tim Nolan (www.timnolan.com).

This is an open source project and you are free to use any code you like.
Please give credit.

Improvements in this version:
- using 2104 MOSFET driver for high side drive
- synchronous operation with high and low side MOSFETS for better buck converter efficiency
- dual high side MOSFETS arranged back to back to prevent current leakage in low light
- arduino protection with zener diodes
- analog input filtering for more accurate measurments
- eliminated absorb state for simpler operation
- eliminated ammeter to reduce cost

Specs:
- 250W input capacity (with heatsinks and fan)
- 50V input max
- 20V output max
- 30A output max  (with heatsinks and fan)
- 30A load capacity (with heatsinks and fan)

Warning!:
-Disconnecting the battery while in a charging state will cause and overshoot of voltage on the battery side.
This could damage any loads that are running from the battery, including the arduino, charge controller, 
and computer (if it's connected at the time)
-Setting the pulseWidth to values less than 40% for even a few milliseconds will cause the low side MOSFET
to short out and fail (sometimes violently).

This code requires that you have these dependancies:
https://arduino-pwm-frequency-library.googlecode.com/files/Arduino%20PWM%20Frequency%20Library%20v_05.zip
