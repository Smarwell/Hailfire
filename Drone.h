#pragma once

#define MODE_MANUAL				0
#define MODE_RETURNING			1
#define MODE_RETURNING_LANDING	2
#define MODE_LANDING			3
#define MODE_LANDED				4
#define MODE_HOLD_POS			5
#define MODE_WANDER				6

#include <Arduino.h>
#include "Motors\Servo.h"
#include "MPU\MPU.h"


class Drone {
	MPU mpu;
	int servo_pins[4] = { 2,3,4,5 };
	Servos servos;
	int flight_mode;
	bool ready;

public:
	Drone(): servos(servo_pins) {};
	bool is_ready();
	bool init();
	void set_mode(uint8_t mode);
	void set_thrust(uint8_t arg);
	void update();
	void check_comms();

};
