
/*

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

*/

//const int min_pulse_width=1100;
//const int max_pulse_width=2000;

/*
Port A 1 - pin 22
Port A 2 - pin 23
Port A 3 - pin 24
Port A 4 - pin 25

OCR4A/B/C
OCR5A		- output compare registers

TIMSK4/5	- timer interrupt


*/



bool motor_update_queue = false;
int motor_queue[4];
int new_motor_queue[4];
int motor_queue_progress = 0;

class ExpServos {
	int pulse_width_compares[];

public:

	ExpServos() {
		//OCR4A = [desired compare match timer count]

		//look into fast PWM
		DDRA |= 0b00001111;

		TCCR5B |= 0b1;
		TCCR4B |= 0b1;
		TIMSK4 |= (1 << OCIE4A) | (1 << OCIE4B) | (1 << OCIE4C);
		TIMSK5 |= (1 << OCIE5A);
	}

};

ISR(TIMER4_COMPA_vect) {

}

ISR(TIMER4_COMPB_vect) {

}

ISR(TIMER4_COMPC_vect) {

}

ISR(TIMER5_COMPA_vect) {

}