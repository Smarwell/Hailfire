
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
	case RESET_VEL:
	case RESET_ALL:
	case COMM_CHECK:
		send_message(COMM_CHECK, 0, "Communications check");
		break;
	case RESET:
	case POWER_OFF:
	case FAST_LAND:
	case SET_MANUAL:
		drone.set_mode(MODE_MANUAL);
		break;
	case SET_AUTO:
		drone.set_mode(MODE_HOLD_POS);
		break;
	case SET_THRUST:
	case SET_YAW:
	default:
		send_message(WARN, INVALID_INPUT, "Bad input given");
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


