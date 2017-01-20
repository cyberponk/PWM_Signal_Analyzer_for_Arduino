// PWM Signal Analyzer for Arduino Uno/Nano
//	Author: Cyberponk
//	V0.1		19/01/2017
//
//	For those who need to analyze a PWM signal for its characteristics, this tool might help.
//
//	Measures frequency from 0,001863 Hz (at 50% duty cycle and overflow counter on)
//					   up to  105263 Hz (at 50% duty cycle and overflow counter off)
//	Measures high, low and total pulse time
//	Measures PWM duty in %
//	Measures PWM output value from 0 to 255 (arduino PWM)
//
//	In the absense of pulsed signals:
//		Measures voltage (assuming the correct voltage divider resistors are used)
//		Measures digital HIGH or LOW signal
//
//	Uses LCD Keypad Shield with LCD 1602 display and buttons.
//	Use Up/Down buttons to switch between different information pages
//  Use Select button to pause reading and freeze information on screen
//  Use left/right buttons to +- pwm output on PWM output pin
//
//	Pins:
//		Pin D2 and A1	-> PWM or Frequency and Digital/Analog Input (connect both pins together after voltage divider resistors)
//		Pin 11			-> 5V PWM Output
//


// Uses LCD Keypad Shield
#include <LiquidCrystal.h> // LCD library
#include <TaskScheduler.h> // Cooperative multitasking library for Arduino by Anatoli Arkhipenko

//==========================================================
// USER OPTIONS
	// ENABLE TIMER1 OVERFLOW COUNTER
	// Without TIMER1 Overflow counter: 	from 122,07Hz to 105263Hz 	(high/low pulses from  4095us to 4,75us	 @ 50% PWM, from 65535 to 75 cycles)
	// With TIMER1 overflow counter:		from 0,001863Hz to 59701Hz	(high/low pulses from 268,43s to 8,375us @ 50% PWM, from 4294967295 to 134 cycles)
	#define USE_T1_OVERFLOW_COUNT // Comment out if not needed

	// Maximum time to show PWM information page before changing to digital/analog mode, when no pulse is read
	//	The program will continue to wait for the pulse in the background, but information will only be shown after
	//	the complete PWM pulse is processed. Default: 10000000 (10 seconds)
	#define PWM_PAGE_TIMEOUT_WHEN_NO_PULSE_US 10000 //[ms]

	#define PULSE_CAPTURE_PERIOD_MS 400	// [ms] Time between each pulse measurement
	#define INFO_UPDATE_PERIOD_MS   PULSE_CAPTURE_PERIOD_MS	//[ms] Time between LCD/Serial updates

	// Changes the PWM output frequency
	// 1 = 31372,55Hz / 2 = 3921,16Hz / 3 = 980,39Hz / 4 = 490,2Hz
	// 5 = 245,1Hz / 6 = 122,55Hz / 7 = 30,64Hz
	#define PWM_OUTPUT_FREQUENCY 7


	#define SERIAL_BAUD 115200

//==========================================================
// PROGRAM DEFINES
	#define INPUTMODE_DISPLAY_PAGES 3
	#define OUTPUTMODE_DISPLAY_PAGES 3

//==========================================================
// PIN ASSIGNMENTS
#define PIN_PWM_READ 2
#define PIN_ANALOG_READ A1
#define POUT_PWM_WRITE 11

//==========================================================
// VARIABLES

	int display_page = 0;
	boolean paused = false;
	boolean btn_handled = false;
	unsigned long btn_millis = 0;

	#ifdef USE_T1_OVERFLOW_COUNT
		#define t1_TCNT_type				unsigned int		// Size for the Timer1 counter and Timer1 Overflow counter
		#define t1_full_counter_type		unsigned long	// 2x the size of t1_TCNT_type

		volatile t1_TCNT_type t1_Overflow_Counter = 0;

		#define pwm_pulse_type double
	#else
		#define t1_TCNT_type				unsigned int		// Size for the Timer1 counter and Timer1 Overflow counter
		#define t1_full_counter_type		t1_TCNT_type		// 2x the size of t1_TCNT_type

		#define pwm_pulse_type unsigned long
	#endif

	volatile t1_full_counter_type t1_Counter_on_INT0 = 0;
	volatile t1_full_counter_type pwm_high_start = 0;
	volatile t1_full_counter_type pwm_low_start = 0;
	volatile t1_full_counter_type pwm_low_end = 0;

	volatile byte pwm_capture_state = 0;
	volatile bool pulse_capture_started = false;
	volatile bool pulse_capture_timeout = true;
	volatile bool pulse_capture_complete = false;

	pwm_pulse_type pwm_pulse = 0;
	pwm_pulse_type pwm_high = 0;
	pwm_pulse_type pwm_low = 0;
	double pwm_value;
	double pwm_duty;
	double pwm_freq;

	int analog_value;
	bool digital_value;
	float voltage_0to5;
	float voltage_0to14;

	byte pwm_out_value = 127; // 0 to 255

//==========================================================
// FUNCTION DECLARATIONS
	void PrintInfoScheduler_callback();

	void WelcomeMessage();

//==========================================================
// CLASSES
	Scheduler runner;
	Task PulseCaptureScheduler(PULSE_CAPTURE_PERIOD_MS, TASK_FOREVER, &PulseCaptureScheduler_callback);
	Task PrintInfoScheduler(INFO_UPDATE_PERIOD_MS, TASK_FOREVER, &PrintInfoScheduler_callback);

	LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


void setup() {
	Serial.begin(SERIAL_BAUD);
	lcd.begin(16, 2); // LCD's columns, rows
	pinMode(10, OUTPUT); // LCD Backlight
	digitalWrite(10,HIGH); // Turn on LCD Backlight

	pinMode(PIN_PWM_READ, INPUT);
	pinMode(PIN_ANALOG_READ, INPUT);
	pinMode(POUT_PWM_WRITE, OUTPUT);

	//WelcomeMessage();

	// Setup Timer1 for precise timing
	TCCR1A = 0; // No comparator, no Waveform generator
	TCCR1B = (1 << CS10); // clk/1 prescaler for 0.0625us precision
//	TCCR1B = (1 << CS11); // clk/8 prescaler for 0.5us precision
	TIMSK1 = 0; // Disable overflow interrupt
	#ifdef USE_T1_OVERFLOW_COUNT
		TIMSK1 = 1; // Enable overflow interrupt (only needed for low/high pulses > 32768 ms)
	#endif

	runner.init();
	runner.addTask(PrintInfoScheduler);
	runner.addTask(PulseCaptureScheduler);
	runner.enableAll();

	TCCR2B = (TCCR2B & 0b11111000) | PWM_OUTPUT_FREQUENCY; // Adjust PWM output frequency using TIMER2 for testing
	analogWrite(POUT_PWM_WRITE, pwm_out_value);

}

void loop() {
	runner.execute();

	int btn = analogRead(0);
	if (btn < 800) {
		if (btn_handled == false) {
			// Button was pressed, find out which one
			if (btn < 60) { // Right
				if (pwm_out_value < 255) pwm_out_value++;
			}
			else if (btn < 200) { // Up
				display_page++;
				if (display_page >(INPUTMODE_DISPLAY_PAGES - 1)) display_page = 0;
				lcd.clear();
			}
			else if (btn < 400){ // Down
				display_page--;
				if (display_page < 0) display_page = INPUTMODE_DISPLAY_PAGES - 1;
				lcd.clear();
			}
			else if (btn < 600){ // Left
				if (pwm_out_value > 0) pwm_out_value--;
			}
			else if (btn < 800){ // Select
				if (paused == true) paused = false;
				else paused = true;
			}

			analogWrite(POUT_PWM_WRITE, pwm_out_value);
			PrintInfoScheduler_callback();

//			Serial.print("btn mls	");Serial.println(btn_millis);
			if (btn_millis == 0) btn_millis = millis();
			btn_handled = true;
		}
		else { // Still pressed
			unsigned long x = millis() - btn_millis;
//			Serial.print("x	");Serial.println(x);
						if (x >  300 && (x % 100) == 0) btn_handled = false; // if btn held for Ams (x > A), repeat pulse every Bms (x % B)
						if (x > 2500 && (x %  10) == 0) btn_handled = false; // if btn held for Ams (x > A), repeat pulse every Bms (x % B)
		}
	}
	else {
		// button Released
		btn_millis = 0;
		btn_handled = false;
	}
}

//==========================================================
// INTERRUPT HANDLERS

#ifdef USE_T1_OVERFLOW_COUNT
	ISR(TIMER1_OVF_vect) // timer1 16-bits - overflows every 65536 counts
	{ t1_Overflow_Counter++;	}
#endif

ISR(INT0_vect) {// External Interrupt pin 2 [D2]
	t1_Counter_on_INT0 = TCNT1;

	#ifdef USE_T1_OVERFLOW_COUNT
		t1_TCNT_type t1_Overflow_Counter_copy = t1_Overflow_Counter; // [4cycles]
		// if just missed an overflow
		if ((TIFR1 & bit(TOV1)) && t1_Counter_on_INT0 < 0x7FFF) t1_Overflow_Counter_copy++; // [3cycles]

		// t1_Counter_on_INT0_HighByte is a pointer to high bytes of t1_Counter_on_INT0
		t1_TCNT_type *t1_Counter_on_INT0_HighBytes = (t1_TCNT_type*)&t1_Counter_on_INT0 + 1;
		*t1_Counter_on_INT0_HighBytes = t1_Overflow_Counter_copy; // // Copy overflow counter to high bytes [4cycles]
	#endif

	if ( paused ) return;

	// 1 pulse = 3 interrupts
	// First interrupt  = rising edge:  high state starts
	// Second interrupt = falling edge: high state finishes and low state starts
	// Third interrupt  = rising edge:  low state finishes
	switch( pwm_capture_state ) {
		case 0: // First rise captured
			pwm_high_start = t1_Counter_on_INT0;
			pwm_capture_state = 1;
			EICRA = 0b0; // set INT0 to trigger on LOW
			//bitClear(EICRA, ISC00); // set INT0 to trigger on FALLING
		break;
		case 1: // Fall captured
			pwm_low_start = t1_Counter_on_INT0;
			pwm_capture_state = 2;
			EICRA = 0b01; // set INT0 to trigger on CHANGE
			//bitSet(EICRA, ISC00); // set INT0 to trigger on RISING
		break;
		case 2: // Second rise captured - Pulse capture complete
			pwm_low_end = t1_Counter_on_INT0;
			pulse_capture_started = false;
			pulse_capture_timeout = false;
			pulse_capture_complete = true;

			bitClear(EIMSK, INT0);  // Turn off INT0
		break;
	}
}

//==========================================================
// SCHEDULER CALLBACKS
void PulseCaptureScheduler_callback() {
//	Serial.println("PCH");

	if( pulse_capture_started ) { // Pulse capture already started, check for timeout
		unsigned char sreg = SREG; // Save Global Interrupt Flag;
		cli(); // Disable interrupts
  		 t1_full_counter_type t1_Counter_now = TCNT1; //Check datasheet for how to read 16-bit registers atomically
		 #ifdef USE_T1_OVERFLOW_COUNT
			t1_TCNT_type t1_Overflow_Counter_copy = t1_Overflow_Counter; // [4cycles]
			// if just missed an overflow
			if ((TIFR1 & bit(TOV1)) && t1_Counter_now < 0x7FFF) t1_Overflow_Counter_copy++; // [3cycles]

			// t1_Counter_on_INT0_HighByte is a pointer to high bytes of t1_Counter_now
			t1_TCNT_type *t1_Counter_now_HighBytes = (t1_TCNT_type*)&t1_Counter_now + 1;
			*t1_Counter_now_HighBytes = t1_Overflow_Counter_copy; // Copy overflow counter to high bytes [4cycles]

		 #endif
		SREG = sreg; // Restore interrupts

		//TODO: Verify PWM_PAGE timeout
		if( (t1_Counter_now - t1_Counter_on_INT0 ) > PWM_PAGE_TIMEOUT_WHEN_NO_PULSE_US *16000) { //pulse timeout
			noInterrupts();
			 pulse_capture_timeout = true;
			interrupts();
//	Serial.print("TO	");Serial.println((micros() - us ));
		}

	} else { // Start a pulse capture
		pwm_high_start = 0;
		pwm_low_start = 0;
		pwm_low_end = 0;
		pwm_capture_state = 0;
		pulse_capture_started = true;
		pulse_capture_complete = false;

//	Serial.print("PCS	");Serial.println("");
		// Turn on external interrupt
		bitSet(EIFR, INTF0); // Clear INT0 interrupt flag (cleared with 1)
		bitSet(EICRA, ISC01); bitSet(EICRA, ISC00); // set INT0 to trigger on RISING
		bitSet(EIMSK, INT0);  // Turn on INT0
	}
}

void PrintInfoScheduler_callback() {
	if( pulse_capture_timeout ) { // No PWM on input pin
		// Print Analog info
		if ( !paused ) {
			analog_value = analogRead(PIN_ANALOG_READ);
			digital_value = digitalRead(PIN_ANALOG_READ);
			voltage_0to5 = 5 * analog_value / 1023.0;
			voltage_0to14 = 14 * analog_value / 1023.0;
		}

		switch (display_page){
		case 0:
			lcd.setCursor(1, 0);
			lcd.print("Analog val:");
			if (analog_value < 1000) lcd.print(" ");
			if (analog_value < 100) lcd.print(" ");
			if (analog_value < 10) lcd.print(" ");
			lcd.print(analog_value);

			lcd.setCursor(1, 1);
			lcd.print("Digital:   ");
			lcd.setCursor(12, 1);
			if( digital_value ) lcd.print("HIGH"); else lcd.print(" LOW");
		break;
		case 1:
			lcd.setCursor(1, 0);
			lcd.print("For 5V scale:  ");

			lcd.setCursor(1, 1);
			lcd.print("          ");
			lcd.print(voltage_0to5, 2);
			lcd.print("V");
		break;
		case 2:
			lcd.setCursor(1, 0);
			lcd.print("For 14V scale: ");

			lcd.setCursor(1, 1);
			lcd.print("         ");
			if (voltage_0to14 < 10) lcd.print(" ");
			lcd.print(voltage_0to14, 2);
			lcd.print("V");
		break;
		}

		Serial.print("ANALOG INSPECTOR");
		Serial.println("");
		Serial.print(" - Analog value[0-1023]: "); Serial.println( analog_value );
		Serial.print(" - Digital value: "); if( digital_value ) Serial.println("HIGH"); else Serial.println(" LOW");
		Serial.print(" - For 5V scale: "); Serial.println(voltage_0to5, 2);
		Serial.print(" - For 14V scale: "); Serial.println(voltage_0to14, 2);

	} else { // PWM on input pin was captured
		// Print PWM inspector info
		if ( !paused && pulse_capture_complete ) {
			//TODO: Verify pulses when timer overflows between readings

/*			if( pwm_high_start > pwm_low_start ) { // Timer overflowed between high_start and low_start
				pwm_low_start += 65536;
				pwm_low_end += 65536;
			}
			if( pwm_low_start > pwm_low_end ) { // Timer overflowed between low_start and low_end
				pwm_low_end += 65536;
			}
*/

			pwm_high = pwm_low_start - pwm_high_start;		// 1 = 0.625us
			pwm_low = pwm_low_end - pwm_low_start;			// 1 = 0.625us
			pwm_pulse = (double)pwm_high + (double)pwm_low;	// 1 = 0.625us

	// used for debugging
	volatile unsigned long x = pwm_high;
	volatile unsigned long y = pwm_low;
	volatile unsigned long z = pwm_pulse;


			pwm_freq = 16000000.0 / pwm_pulse; // 2000000 for clk/8, 16000000 for clk/1 timer1 prescaler
			pwm_duty = (100 * pwm_high) / (double)pwm_pulse;
			pwm_value = (255 * pwm_high ) / (double)pwm_pulse;
		}
		//TODO: Format LCD outputs
		switch (display_page){
		case 0:
			lcd.setCursor(1, 0);
			lcd.print("Freq:");
			if (pwm_freq < 10000) lcd.print(" ");
			if (pwm_freq < 1000) lcd.print(" ");
			if (pwm_freq < 100) lcd.print(" ");
			if (pwm_freq < 10) lcd.print(" ");
			lcd.print(pwm_freq, 2);
			lcd.print("Hz");

			lcd.setCursor(1, 1);
			lcd.print("Duty:    ");
			if (pwm_duty < 10) lcd.print(" ");
			lcd.print(pwm_duty, 2);
			lcd.print("%");
		break;
		case 1:
			lcd.setCursor(1, 0);
			lcd.print("Pulse:");
			unsigned long pwm_pulse_tmp;
			pwm_pulse_tmp = (unsigned long) pwm_pulse/2;
			if (pwm_pulse_tmp < 10000) lcd.print(" ");
			if (pwm_pulse_tmp < 1000) lcd.print(" ");
			if (pwm_pulse_tmp < 100) lcd.print(" ");
			if (pwm_pulse_tmp < 10) lcd.print(" ");
			lcd.print(pwm_pulse/2.0, 1);
			lcd.print("us");

			lcd.setCursor(1, 1);
			lcd.print("PWM val: ");
			if (pwm_value < 100) lcd.print(" ");
			if (pwm_value < 10) lcd.print(" ");
			lcd.print(pwm_value,2);
		break;
		case 2:
			lcd.setCursor(1, 0);
			lcd.print("High:");
			unsigned long pwm_high_tmp = pwm_high/2;
			if (pwm_high_tmp < 100000) lcd.print(" ");
			if (pwm_high_tmp < 10000) lcd.print(" ");
			if (pwm_high_tmp < 1000) lcd.print(" ");
			if (pwm_high_tmp < 100) lcd.print(" ");
			if (pwm_high_tmp < 10) lcd.print(" ");
			lcd.print(pwm_high/2.0, 1);
			lcd.print("us");

			lcd.setCursor(1, 1);
			lcd.print("Low:");
			unsigned long pwm_low_tmp = pwm_low/2;
			if (pwm_low_tmp < 1000000) lcd.print(" ");
			if (pwm_low_tmp < 100000) lcd.print(" ");
			if (pwm_low_tmp < 10000) lcd.print(" ");
			if (pwm_low_tmp < 1000) lcd.print(" ");
			if (pwm_low_tmp < 100) lcd.print(" ");
			if (pwm_low_tmp < 10) lcd.print(" ");
			lcd.print(pwm_low/2.0, 1);
			lcd.print("us");
		break;
		}

		Serial.print("PWM INSPECTOR");
		Serial.println("");
		Serial.print(" - Pulse[ms]: "); Serial.println(pwm_pulse/2.0, 2);
		Serial.print(" - Time HIGH/LOW [ms]: "); Serial.print(pwm_high/2.0, 2); Serial.print(" / "); Serial.println(pwm_low/2.0, 2);
		Serial.print(" - Duty[%]: "); Serial.println(pwm_duty, 2);
		Serial.print(" - Calculated PWM byte value: "); Serial.println(pwm_value);
		Serial.print(" - Freq[kHz]:"); Serial.println(pwm_freq, 2);

	}

	if ( paused ) {
		lcd.setCursor(0, 0); lcd.print("*");
		Serial.println("PAUSED");

	} else {
		lcd.setCursor(0, 0); lcd.print(" ");
	}

	Serial.println("");
}


//==========================================================
// OTHER FUNCTIONS

void WelcomeMessage() {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("PWM Inspector");
	lcd.setCursor(0, 1);
	lcd.print("Baud Rt: "); lcd.print(SERIAL_BAUD);
	delay(2000);

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Pin in: "); lcd.print(PIN_PWM_READ);
	lcd.setCursor(0, 1);
	lcd.print("Test pin: "); lcd.println(POUT_PWM_WRITE);
	delay(2000);
	lcd.clear();
}
