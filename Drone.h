#pragma once

const float YAW_P = 0;
const float YAW_I = 0;
const float YAW_D = 0;
const float PITCH_P = 0;
const float PITCH_I = 0;
const float PITCH_D = 0;
const float ROLL_P = 0;
const float ROLL_I = 0;
const float ROLL_D = 0;

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
	bool ready;

	PID yaw_controller;
	PID pitch_controller;
	PID roll_controller;

public:
	Drone() :
		servos(),
		mpu(),
		yaw_controller(YAW_P, YAW_I, YAW_D),
		pitch_controller(PITCH_P, PITCH_I, PITCH_D),
		roll_controller(ROLL_P, ROLL_I, ROLL_D) {};
	void init();
	void set_mode(uint8_t mode);
	void set_thrust(uint8_t arg);
	void update();
	void periodic_update();
	void reset_gyro_setpoint(int);
	void reset_gyro_setpoints();
	void reset_vel();
	void reset_gyro();
	void set_gyro_setpoint(int,float);

	bool is_ready() { return ready; }
};
