
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

bool relevant(uint8_t command) {
	return command <= COMM_SET_AUTO || autonomous_flight ^ (command < COMM_RETURN_START);
}

void send_message(uint8_t type, uint8_t data = 0, String message="") { 
	if (debug) {
		Serial.println(message);
	}
	else {
		Serial.write(type << 8 || data);
	}
}

void send_telem() {

}

void comm_parse() {
	switch (comm_command) {
	case COMM_RESET_VEL:
	case COMM_RESET_ALL:
	case COMM_CHECK:
		send_message(COMM_CHECK, 0, "Communications check");
		break;
	case COMM_SET_MANUAL:
		drone.set_mode(MODE_MANUAL);
		break;
	case COMM_SET_AUTO:
		drone.set_mode(MODE_HOLD_POS);
		break;
	case COMM_SET_THRUST:
	case COMM_SET_X_VEL:
	case COMM_SET_Y_VEL:
	case COMM_SET_Z_VEL:
	case COMM_SET_X_POS:
	case COMM_SET_Y_POS:
	case COMM_SET_Z_POS:
	case COMM_SET_YAW:
	case COMM_RETURN_START:
	case COMM_RETURN_LAND:
	case COMM_LAND:
	case COMM_TAKE_OFF:
	case COMM_HOLD_POS:
	case COMM_WANDER:
	default:
		send_message(COMM_WARN, INVALID_INPUT, "Bad input given");
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
		if (relevant(comm_command)) {
			comm_parse();
		}
	}
}


