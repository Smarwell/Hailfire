
#include "Comm.h"
#include "Drone.h"

Drone drone;

/*
Communications between the drone and computer follow a simple format.
Each message is normally two bytes, with the first being a identifier
for what kind of command or message is being sent, and the second byte
is the content of the message. For example, (5,127) is a command to
set the baseline thrust to 127/255, or about 50%.
*/

uint8_t comm_command, comm_arg;
uint8_t output_buffer[28];

void send_message(uint8_t message, String str="") { 
	if (debug) {
		Serial.println(str);
	}
	else {
		Serial.write(MESSAGE << 8 || message);
	}
}

void send_telem() {

}

void comm_parse() {
	switch (comm_command) {
	case RESET_VEL:
		drone.reset_vel();
		break;
	case RESET_ALL:
		drone.reset_gyro();
		drone.reset_vel();
		break;
	case COMM_CHECK:
		send_message(COMM_CHECK, "Communications check");
		break;
	case RESET:
		break;
	case POWER_OFF:
		drone.set_thrust(0);
		break;
	case FAST_LAND:
		drone.set_thrust(60); //~25%
		break;
	case SET_MANUAL:
		drone.set_mode(MANUAL);
		break;
	case SET_AUTO:
		drone.set_mode(HOLD_POS);
		break;
	case SET_THRUST:
		drone.set_thrust(comm_arg);
		break;
	case SET_YAW:
		drone.set_gyro_setpoint(0,(float(comm_arg) - 127) / 127.0*M_PI);
		break;
	case SET_PITCH:
		drone.set_gyro_setpoint(1, (float(comm_arg) - 127) / 127.0*M_PI / 2);
		break;
	case SET_ROLL:
		drone.set_gyro_setpoint(1, (float(comm_arg) - 127) / 127.0*M_PI / 2);
		break;
	default:
		send_message(INVALID_INPUT, "Bad input given");
		break;
	}
}

bool comm_check() {
	if (debug) return true;
	send_message(COMM_CHECK);
	int time = millis();
	while (Serial.available() == 0 && millis() - time < 500) {
		delay(1);
	}
	return (millis() - time < 500 && Serial.read() == COMM_CHECK);
}

void check_for_message() {
	if (Serial.available() > 1) {
		comm_command = Serial.read();
		comm_arg = Serial.read();
		comm_parse();
	}
}


