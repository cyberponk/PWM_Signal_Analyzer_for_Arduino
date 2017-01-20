 PWM Signal Analyzer for Arduino Uno/Nano
 Author: Cyberponk
 V0.1		19/01/2017

 For those who need to analyze a PWM signal for its characteristics, this tool might help.

 	Measures frequency from 0,001863 Hz (at 50% duty cycle and overflow counter on)
					   up to  105263 Hz (at 50% duty cycle and overflow counter off)
 	Measures high, low and total pulse time
 	Measures PWM duty in %
 	Measures PWM output value from 0 to 255 (arduino PWM)

 In the absense of pulsed signals:

  	Measures voltage (assuming the correct voltage divider resistors are used)
  	Measures digital HIGH or LOW signal

	Uses LCD Keypad Shield with LCD 1602 display and buttons.
 	Up/Down buttons to switch between different information pages
	Select button to pause reading and freeze information on screen
	Left/Right buttons to +- pwm output on PWM output pin

 Pins:

  	Pin D2 and A1	-> PWM or Frequency and Digital/Analog Input (connect both pins)
  	Pin 11			-> 5V PWM Output

