
/*
This code replaces the old motor control code. It hijacks the fifth hardware timer 
on the Mega to drive the motors through interrupts.

This code will only run on the Arduino Mega 2560, and it *will* break anything that 
relies on hardware timer 5 to run, including PWM for pins 44-46 and the builtin 
Servo library.

+----+      +----+
| 0  |  /\  | 1  |
| NW | /||\ | NE |
+----+  ||  +----+
      \ || /
	   \||/
	    \/
		/\
	   /||\
	  / || \
+----+  ||  +----+
| 2  |  ||  | 3  |
| SW |      | SE |
+----+      +----+


Port A 0 - pin 22
Port A 1 - pin 23
Port A 2 - pin 24
Port A 3 - pin 25

Registers used in this code:

DDRA	- Sets the mode for the pins of Port A - 0 is read, 1 is write
PORTA	- Sets the output for the pins of Port A

TCCR5A	- Timer/Counter configuration register B, timer 5 - controls the mode the timer runs in.
TCCR5B	- Timer/Counter configuration register B, timer 5 - controls the mode the timer runs in.
TIMSK5	- Timer Interrupt Mask, timer 5 - controls what triggers an interrupt
OCR5A	- Compare match register, timer 5, channel A - controls for how many cycles the timer runs 
													   before triggering an interrupt
*/

#include "Arduino.h"

const uint16_t min_compare = 17693;	//1100 microseconds * ~16 clock cycles per microsecond
const uint16_t max_compare = 32058;	//2000 microseconds
									//The exact values were determined experimentally

uint8_t motor_queue_progress = 0;
uint16_t motor_compares[];

class Servos {

	//Converts a power level from 0 to 1 to a number of clock cycles
	uint16_t power_to_compare(float power) {
		return uint16_t(power*(max_compare - min_compare));
	}

	//Makes sure the motor power never goes above 100% or below 0%
	void check_power_level(int motor) {
		if (motor_compares[motor] > max_compare) motor_compares[motor] = max_compare;
		if (motor_compares[motor] < min_compare) motor_compares[motor] = min_compare;
	}

public:

	Servos() {};

	//Configures the timer and sets up the motors
	void start() {
		DDRA |= 0b00001111;
		//set pins 22-25 to output

		TCCR5A = 0b00000000;
		TCCR5B = 0b00001001;
		//set the timer to run without a prescaler in CTC mode.

		TIMSK5 |= (1 << OCIE5A);
		//tell it to trigger the interrupt on compare match

		set_power(0, 0);
		set_power(1, 0);
		set_power(2, 0);
		set_power(3, 0);
	}


	void set_power(int motor, float power) {
		motor_compares[motor] = power_to_compare(power) + min_compare;
		check_power_level(motor);
	}

	void modify_power(int motor, float dpower) {
		motor_compares[motor] += power_to_compare(dpower);
		check_power_level(motor);
	}

	void manip_motors(int positive1, int positive2, int negative1, int negative2, float factor) {
		modify_power(positive1, factor);
		modify_power(positive2, factor);
		modify_power(negative1, -factor);
		modify_power(negative2, -factor);
	}

};

//called whenever one motor's pulse needs to be turned off, and the next one
//turned on.
ISR(TIMER5_COMPA_vect) {
	PORTA &= 0b11110000;
	//Turn off all four motor pins

	motor_queue_progress = (motor_queue_progress + 1) % 4;
	//advance to the next motor in the queue

	OCR5A = motor_compares[motor_queue_progress];
	//set the compare target to the next value in the queue

	PORTA |= 0b1 << motor_queue_progress;
	//turn on the corresponding motor
}
