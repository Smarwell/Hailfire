#pragma once

enum modes {
	MANUAL,
	RETURNING,
	RETURNING_LANDING,
	LANDING,
	LANDED,
	HOLD_POS,
	WANDER
};

#include <Arduino.h>
#include "Motors\Servo.h"
#include "MPU\MPU.h"
#include "PID.h"
#include "Comm.h"


class Drone {
	MPU mpu;
	Servos servos;
	int flight_mode;

public:
	Drone(): servos() {};
	bool init();
	void set_mode(uint8_t mode);
	void set_thrust(uint8_t arg);
	void update();
	void periodic_update();
	void reset_gyro_setpoint(int);
	void reset_gyro_setpoints();
	void reset_vel();
	void reset_gyro();
	void set_gyro_setpoint(int,float);
};
