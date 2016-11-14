/*
 Name:		ArduinoControlExploration.ino
 Created:	9/15/2016 12:54:44 PM
 Author:	Jacob Sacco
*/

#include "Drone.h"
#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include "Motors\Servo.h"
#include "MPU\MPU.h"

Drone drone;
int update_counter = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	Serial.setTimeout(50);
	drone.init();
}

// the loop function runs over and over again until power down or reset
void loop() {
	if (drone.is_ready()) {
		drone.update();
		update_counter++;
		if (update_counter % 100 == 0) {
			drone.periodic_update();
		}
	}
}
