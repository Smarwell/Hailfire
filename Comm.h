#pragma once

#define COMM_RESET_VEL 0
#define COMM_RESET_ALL 1
#define COMM_CHECK 2
#define COMM_SET_MANUAL 3
#define COMM_SET_AUTO 4
#define COMM_SET_THRUST 5
#define COMM_SET_X_VEL 6
#define COMM_SET_Y_VEL 7
#define COMM_SET_Z_VEL 8
#define COMM_SET_X_POS 9
#define COMM_SET_Y_POS 10
#define COMM_SET_Z_POS 11
#define COMM_SET_YAW 12
#define COMM_RETURN_START 13
#define COMM_RETURN_LAND 14
#define COMM_LAND 15
#define COMM_TAKE_OFF 16
#define COMM_HOLD_POS 17
#define COMM_WANDER 18

#define COMM_WARN 0
#define COMM_TELEM 1
//		COMM_CHECK 2

#define INVALID_INPUT 0




bool autonomous_flight;
uint8_t comm_command, comm_arg;
uint8_t output_buffer[28];

bool relevant(uint8_t command) {
	return command <= COMM_SET_AUTO || autonomous_flight ^ (command < COMM_RETURN_START);
}

void send_message(uint8_t type, uint8_t data=0) {
	Serial.write(type << 8 || data);
}

void send_telem() {

}

void comm_parse() {
	switch (comm_command) {
	case COMM_RESET_VEL:
	case COMM_RESET_ALL:
	case COMM_CHECK:
	case COMM_SET_MANUAL:
	case COMM_SET_AUTO:
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
	}
}

bool comm_check() {
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










