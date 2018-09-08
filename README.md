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

Physical Improvements:
- Larger copper cross section in high current paths
- 20ohm resistors in series on the MOSFET gates
- added pull-up and pull-down resistors to prevent undesireable behaviour on startup
- Larger inductor 20µH 26A peak
- added fan
- removed diode between high side MOSFET gates
- Constant voltage method of MPPT that doesn't require current sense
- Zener diodes to protect the arduino from over-voltage

Warning!:
-Disconnecting the battery while in a charging state will cause an overshoot of voltage on the battery side.
This could damage any loads that are running from the battery, including the arduino, charge controller, 
and computer (if it's connected at the time)
-Setting the pulseWidth to values less than 40% for even a few milliseconds will cause the low side MOSFET
to short out and fail (sometimes violently).

This code requires that you have these dependancies:
https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads


