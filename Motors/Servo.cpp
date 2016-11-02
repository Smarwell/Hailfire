//For the Hailfire bot
//Author Jacob Sacco


#include "Servo.h"
#include <Arduino.h>

////////////////////////////////////////////////////
////    Control code for an individual servo    ////
////////////////////////////////////////////////////

void Servo::write_pulse_width(int new_width) {
	//Sets a new power level, and sets a flag for the pulse queue to be
	//redone.
	if (new_width > max_pulse_width) new_width = max_pulse_width;
	if (new_width < min_pulse_width) new_width = min_pulse_width;
	pulse_width = new_width;
	power_modified = true;
}

unsigned long int Servo::send_pulse() {
	//sends a new pulse, returns when end_pulse() must be called.
	pulsing = true;
	power_modified = false;
	digitalWrite(pin, true);
	last_pulse_beginning = micros();
	next_pulse_ending = last_pulse_beginning + pulse_width;
	return next_pulse_ending;
}

bool Servo::end_pulse() {
	//ends the current pulse, returns whether or not end_pulse() called in time.
	pulsing = false;
	if (micros() > next_pulse_ending) {
		digitalWrite(pin, false);
		return false;
	}
	while (micros() < next_pulse_ending) {};
	digitalWrite(pin, false);
	return true;
}

bool Servo::get_power_modified() {
	return power_modified;
}

bool Servo::get_pulsing() {
	return pulsing;
}

unsigned long int Servo::get_next_pulse_ending() {
	return next_pulse_ending;
}

Servo::Servo(int pin) : pin{ pin }, pulse_width{ min_pulse_width }, pulsing{ false } {
	pinMode(pin, OUTPUT);
};

void Servo::set_power(double power) {
	//sets a new power level to some proportion of the maximum possible power,
	//in the form of a float between 0 and 1.
	write_pulse_width(int(power*(max_pulse_width - min_pulse_width)) + min_pulse_width);
}

void Servo::modify_power(double dpower) {
	//Modifies the motor's power by some proportion of the maximum possible power,
	//represented as a float between 0 and 1.
	write_pulse_width(int(dpower*(max_pulse_width - min_pulse_width)) + pulse_width);
}

int Servo::get_power() {
	return pulse_width;
}



///////////////////////////////////////////////
////    Control code for all the servos    ////
///////////////////////////////////////////////


void Servos::update_queue() {
	//uses a bubble sort to reorganize the pulse queue based on each motor's
	//pulse width (stored in pulses)
	modified = true;
	while (modified) {
		modified = false;
		for (int i = 0; i < 3; i++) {
			if (pulses[queue[i]] > pulses[queue[i + 1]]) {
				swap_temp = queue[i];
				queue[i] = queue[i + 1];
				queue[i + 1] = swap_temp;
				modified = true;
			}
		}
	}
}

void Servos::new_pulse() {
	//Generates a new pulse for all four motors. If one of the motors' power
	//has been changed since the last pulse, it will call update_queue to
	//make sure all pulses end in the correct order.
	should_update_queue = false;
	for (int i = 0; i < 4; i++) {
		if (servos[i].get_power_modified()) {
			should_update_queue = true;
		}
		pulses[i] = servos[i].send_pulse();
	}
	if (should_update_queue) {
		update_queue();
	}
}

Servos::Servos(int pins[4]) : queue_progress(0), queue{ 0,1,2,3 } {
	for (int i = 0; i < 4; i++) {
		servos[i] = Servo(pins[i]);
		servos[i].set_power(0.0);
	}
}

Servo& Servos::operator[](int i) {
	return servos[i];
}

void Servos::update() {
	//generates a new pulse in all motors, and ends each pulse at the proper time
	//before returning.
	new_pulse();
	for (int i = 0; i < 4; i++) {
		while (micros() < pulses[queue[i]]) {};
		servos[queue[i]].end_pulse();
	}
}

int* Servos::get_power_levels() {
	for (int i = 0; i < 4; i++) {
		power_levels[i] = servos[i].get_power();
	}
	return power_levels;
}

void Servos::write_power_levels(int* new_powers) {
	for (int i = 0; i < 4; i++) {
		servos[i].write_pulse_width(new_powers[i]);
	}
}