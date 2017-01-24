/*
 Name:		ArduinoControlExploration.ino
 Created:	9/15/2016 12:54:44 PM
 Author:	Jacob Sacco
*/

bool led;

#include "Drone.h"
#include "Drone_definitions.h"
#include "Comm_definitions.h"
#include <Wire.h>
#include <I2Cdev.h>

Drone drone;
int update_counter = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	Serial1.begin(9600);
	Serial.setTimeout(50);
	Serial1.setTimeout(50);
	pinMode(2, OUTPUT);
	drone.init();
}

// the loop function runs over and over again until power down or reset
void loop() {
	if (drone.is_ready()) {
		drone.update();
		update_counter++;
		if (update_counter == 100) {
			drone.periodic_update();
			update_counter = 0;
		}
	}
	delay(10);
}
