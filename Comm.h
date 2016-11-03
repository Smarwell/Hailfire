#pragma once

/*
Communications between the drone and computer follow a simple format.
Each message is normally two bytes, with the first being a identifier
for what kind of command or message is being sent, and the second byte
is the content of the message. For example, (5,127) is a command to 
set the baseline thrust to ~50% (127/255).
*/

#include <Arduino.h>

enum comm {
	RESET_VEL,
	RESET_ALL,
	COMM_CHECK,
	RESET,
	POWER_OFF,
	FAST_LAND,
	SET_MANUAL,
	SET_AUTO,
	SET_THRUST,
	SET_X_VEL,
	SET_Y_VEL,
	SET_Z_VEL,
	SET_YAW,
	SET_PITCH,
	SET_ROLL,
	RETURN_START,
	RETURN_LAND,
	LAND,
	TAKE_OFF,
	HOLD_POS,
	WANDER,

	MESSAGE,
	TELEM,
  //COMM_CHECK,

	INVALID_INPUT,
	MPU_INIT_FAILED,
	MPU_FIFO_OVERFLOW,
	MPU_CALIBRATED,
	LOW_BATTERY
};

bool debug = false;

uint8_t comm_command, comm_arg;

//Holds telemetry data [x, y, z, yaw, pitch, roll, distance to wall].
//Not useful yet.
uint8_t output_buffer[28];

//Sends a message to the computer.
void send_message(uint8_t, String);

//Sends telemetry data. Not implemented.
void send_telem();

//Given the command and argument, decide what to do
void comm_parse();

//Check if the drone has good communications with the computer.
bool comm_check();

//Check if there is a message, and parse it if there is.
void check_for_message();

