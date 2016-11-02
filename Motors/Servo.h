#pragma once

//For the Hailfire bot
//Author Jacob Sacco


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

#define S_NW 0
#define S_NE 1
#define S_SW 2
#define S_SE 3

#define min_pulse_width 1000	//microseconds
#define max_pulse_width 2000

class Servo {
	/*
	Represents a motor, and mediates individual pulses, controlled from
	a Servos instance
	*/
	friend class Servos;
	int pulse_width;
	int pin;
	unsigned long int last_pulse_beginning;
	unsigned long int next_pulse_ending;
	bool power_modified;
	bool pulsing;

	void write_pulse_width(int new_width);
	unsigned long int send_pulse();
	bool end_pulse();
	bool get_power_modified();
	bool get_pulsing();
	unsigned long int get_next_pulse_ending();

public:
	Servo() :pin{ -1 } {}
	Servo(int pin);
	void set_power(double power);
	void modify_power(double dpower);

	int get_power();
};



class Servos {
	/*
	A container that mediates software interaction with the motors.

	The instance maintains four Servo instances, one for each motor.
	When update() is called, it generates a single pulse for each motor,
	and will return once the last pulse ends. This can take anywhere from 
	one to two milliseconds.

	May be modified at some point to constantly send pulses, but due to the complexity
	of making that happen, it'll only be done if more performance is needed
	than the current implementation can provide.
	*/
	Servo servos[4];
	int queue[4];
	unsigned long int pulses[4];
	int queue_progress;
	int swap_temp;
	bool modified;
	bool should_update_queue;
	int power_levels[4];

	void update_queue();
	void new_pulse();

public:

	Servos(int pins[4]);
	Servo& operator[](int i);
	void update();
	int* get_power_levels();
	void write_power_levels(int*);
};