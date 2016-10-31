#pragma once

/*
Communications between the drone and computer follow a simple format.
Each message is normally two bytes, with the first being a identifier
for what kind of command or message is being sent, and the second byte
is the content of the message. For example, (5,127) is a command to 
set the baseline thrust to ~50% (127/255).
*/

#include <Arduino.h>

//Things the drone can be sent
//Relevant in both autonomous and manual
#define COMM_RESET_VEL		0
#define COMM_RESET_ALL		1
#define COMM_CHECK			2
#define COMM_SET_MANUAL		3
#define COMM_SET_AUTO		4
//Relevant only in manual
#define COMM_SET_THRUST		5
#define COMM_SET_X_VEL		6
#define COMM_SET_Y_VEL		7
#define COMM_SET_Z_VEL		8
#define COMM_SET_X_POS		9
#define COMM_SET_Y_POS		10
#define COMM_SET_Z_POS		11
#define COMM_SET_YAW		12
//relevant only in autonomous
#define COMM_RETURN_START	13
#define COMM_RETURN_LAND	14
#define COMM_LAND			15
#define COMM_TAKE_OFF		16
#define COMM_HOLD_POS		17
#define COMM_WANDER			18


//Things the drone can send
#define COMM_WARN			0
#define COMM_TELEM			1
//		COMM_CHECK			2

//Error messages
#define INVALID_INPUT		0
#define MPU_INIT_FAILED		1
#define MPU_FIFO_OVERFLOW	2
#define MPU_CALIBRATED		3

bool debug = false;

uint8_t comm_command, comm_arg;

//Holds telemetry data [x, y, z, yaw, pitch, roll, distance to wall].
//Not useful yet.
uint8_t output_buffer[28];

//Checks whether a command is relevant given the drone's current mode
bool relevant(uint8_t command);

//Sends a message to the computer.
void send_message(uint8_t type, uint8_t data = 0, String message = "");

//Sends telemetry data. Not implemented.
void send_telem();

//Given the command and argument, decide what to do
void comm_parse();

//Check if the drone has good communications with the computer.
bool comm_check();

//Check if there is a message, and parse it if there is.
void check_for_message();

