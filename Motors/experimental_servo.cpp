
/*
This code aims to replace the Servo.h and Servo.cpp currently in the project. It
hijacks the fifth hardware timer on the Mega to drive the motors through interrupts.
It does this by setting up a circular "queue" of motors, storing the power level
of each motor in the form of a number of clock cycles for the pulse to stay on.
The timer counts clock cycles, and when it hits the correct number for one motor,
it resets itself, and calls an interrupt that ends that pulse, sets the timer to 
wait for the *next* motor's number of clock cycle, and then turns on that motor's pin.



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


Port A 1 - pin 22
Port A 2 - pin 23
Port A 3 - pin 24
Port A 4 - pin 25

Registers used in this code:

DDRA	- Sets the mode for the pins of Port A - 0 is read, 1 is write
PORTA	- Sets the output for the pins of Port A

TCCR5B	- Timer/Counter configuration register B, timer 5 - controls the mode the timer runs in.
TIMSK5	- Timer Interrupt Mask, timer 5 - controls what triggers an interrupt
OCR5A	- Compare match register, timer 5, channel A - controls for how many cycles the timer runs 
													   before triggering an interrupt
*/

#include "Arduino.h"

const float min_compare = 1100.f * 16.f;	//1100 microseconds
const float max_compare = 2000.f * 16.f;	//16 clock cycles per microsecond

uint8_t motor_queue_progress = 0;
uint16_t motor_compares[];

class ExpServos {

	uint16_t power_to_compare(float power) {
		return uint16_t(power*(max_compare - min_compare));
	}

	void check_power_level(int motor) {
		if (motor_compares[motor] > max_compare) motor_compares[motor] = max_compare;
		if (motor_compares[motor] < min_compare) motor_compares[motor] = min_compare;
	}

public:

	ExpServos() {

		DDRA |= 0b00001111;	
		//set pins 22-25 to output

		TCCR5B = 0b00001001;
		//set the timer to run without a prescaler in CTC mode.

		TIMSK5 |= (1 << OCIE5A);
		//tell it to trigger the interrupt on compare match
	}


	void set_power(int motor, float power) {
		motor_compares[motor] = power_to_compare(power);
		check_power_level(motor);
	}

	void modify_power(int motor, float dpower) {
		motor_compares[motor] += power_to_compare(dpower);
		check_power_level(motor);
	}

};


ISR(TIMER5_COMPA_vect) {
	PORTA &= 0b11110000;
	//Turn off all four motor pins

	motor_queue_progress = (motor_queue_progress + 1) % 4;
	//advance to the next motor in the queue

	OCR5A = motor_compares[motor_queue_progress];
	//set the compare target to the next value in the queue

	PORTA |= 1 << motor_queue_progress;
	//turn on the corresponding motor
}