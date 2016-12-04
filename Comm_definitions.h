#pragma once

#include "Drone.h"

extern Drone drone;

void send_message(uint8_t message, String str = "") {
	if (debug) {
		Serial.println(str);
	}
	else {
		Serial1.write(MESSAGE << 8 || message);
	}
}

void send_telem() {

}

void comm_parse() {
	switch (comm_command) {
	case RESET_VEL:
		break;
	case RESET_ALL:
		drone.reset_gyro();
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
		Serial.println(comm_arg);
		drone.set_thrust(comm_arg);
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
	while (Serial1.available() == 0 && millis() - time < 500) {
		delay(1);
	}
	return (millis() - time < 500 && Serial1.read() == COMM_CHECK);
}

void check_for_message() {
	if (Serial1.available()>1) {
		comm_command = Serial1.read();
		comm_arg = Serial1.read();
		comm_parse();
	}
}