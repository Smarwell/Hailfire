#pragma once

const float YAW_P = .3;
const float YAW_I = 0;
const float YAW_D = 0;

const float PITCH_P = .25;
const float PITCH_I = 0;
const float PITCH_D = 10000;

const float ROLL_P = PITCH_P;
const float ROLL_I = PITCH_I;
const float ROLL_D = PITCH_D;

#include <Arduino.h>
#include "Motors\Servo.h"
#include "MPU\MPU.h"
#include "PID.h"
#include "Comm.h"

enum modes {
	MANUAL,
	RETURNING,
	RETURNING_LANDING,
	LANDING,
	LANDED,
	HOLDING_POS,
	WANDERING
};

class Drone {
	Servos servos;
	MPU mpu;
	int flight_mode;
	bool ready;

	float vel_x;
	float vel_y;
	float vel_z;

	PID yaw_pid;
	PID pitch_pid;
	PID roll_pid;

	bool pid_enabled;

	float res; //For debugging

public:

	Drone() :
		servos(),
		mpu(),
		yaw_pid(YAW_P, YAW_I, YAW_D),
		pitch_pid(PITCH_P, PITCH_I, PITCH_D),
		roll_pid(ROLL_P, ROLL_I, ROLL_D),
		pid_enabled(true){};
	void init();
	void set_mode(uint8_t mode);
	void set_thrust(uint8_t arg);
	void update();
	void periodic_update();
	void reset_pid_controllers();
	void reset_gyro();
	bool is_ready() { return ready; }
	void kill_pid_controllers();
	void reenable_pid_controllers();
};
