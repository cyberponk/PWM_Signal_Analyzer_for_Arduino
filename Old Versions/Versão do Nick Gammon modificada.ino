
// Duty cycle calculation using input capture unit
// Author: Nick Gammon
// Date: 5 November 2013

// Input: Pin D8 

volatile boolean first;
volatile boolean triggered;
volatile unsigned long overflowCount;
volatile unsigned long startTime;
volatile unsigned long finishTime;

// timer overflows (every 65536 counts)
ISR(TIMER1_OVF_vect)
{
	overflowCount++;
}  // end of TIMER1_OVF_vect

ISR(TIMER1_CAPT_vect)
{
	// grab counter value before it changes any more
	unsigned int timer1CounterValue;
	timer1CounterValue = ICR1;  // see datasheet, page 151 (accessing 16-bit registers)

	unsigned long overflowCopy = overflowCount;
	// if just missed an overflow
	if ((TIFR1 & bit(TOV1)) && timer1CounterValue < 0x7FFF)
		overflowCopy++;

	// wait until last pulse has been processed
	if (triggered)
		return;

	if (first)
	{
		startTime = (overflowCopy << 16) + timer1CounterValue;

		TIFR1 |= bit(ICF1);     // clear Timer/Counter1, Input Capture Flag
		TCCR1B = bit(CS10);    // No prescaling, Input Capture Edge Select (falling on D8)
		first = false;
		return;
	}

	finishTime = (overflowCopy << 16) + timer1CounterValue;
	_NOP(); // Removing this causes random errors in startTime

	triggered = true;
	TIMSK1 = 0;    // no more interrupts for now
}  // end of TIMER1_CAPT_vect

void prepareForInterrupts()
{
	noInterrupts();  // protected code
	first = true;
	triggered = false;  // re-arm for next time
	// reset Timer 1
	TCCR1A = 0;
	TCCR1B = 0;

	TIFR1 = bit(ICF1) | bit(TOV1);  // clear flags so we don't get a bogus interrupt
	TCNT1 = 0;          // Counter to zero
	overflowCount = 0;  // Therefore no overflows yet

	// Timer 1 - counts clock pulses
	TIMSK1 = bit(TOIE1) | bit(ICIE1);   // interrupt on Timer 1 overflow and input capture
	// start Timer 1, no prescaler
	TCCR1B = bit(CS10) | bit(ICES1);  // plus Input Capture Edge Select (rising on D8)
	interrupts();
}  // end of prepareForInterrupts


void setup()
{

	TCCR0B = (TCCR0B & 0b11111000) | 0x02;
	analogWrite(5, 1); //976.5625Hz, with high pulses of ~1/976.5625 x 10/256 = 40us; Connect pin 5 to INPUT_PIN and open serial monitor & you will see approximately this
	analogWrite(6, 254); //976.5625Hz, with high pulses of ~1/976.5625 x 10/256 = 40us; Connect pin 5 to INPUT_PIN and open serial monitor & you will see approximately this

	TCCR1B = (TCCR1B & 0b11111000) | 0x01;
	analogWrite(9, 1); //490.20Hz, with high pulses of ~1/490.2 x 128/256 = ~1020us; Connect pin 9 to INPUT_PIN and open serial monitor & you will see approximately this


	Serial.begin(115200);
	Serial.println("Duty cycle width calculator");
	// set up for interrupts
	prepareForInterrupts();
} // end of setup

void loop()
{
	// wait till we have a reading
	if (!triggered)
		return;

	// period is elapsed time
	unsigned long elapsedTime = finishTime - startTime;
	float pulseTime = float(elapsedTime) * 62.5e-9 * 1e6;
	float freq = 1e6 / pulseTime;
	Serial.print(startTime);	Serial.print("	");
	Serial.print(finishTime);	Serial.print("		");

	Serial.print("Took: ");
	Serial.print(pulseTime);  // convert to microseconds
	Serial.print(" uS.		");
	Serial.print(freq);  // convert to microseconds
	Serial.println(" Hz ");

	// so we can read it  
	delayMicroseconds(500000);

	prepareForInterrupts();
}   // end of loop