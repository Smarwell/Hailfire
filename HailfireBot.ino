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

bool autonomous_flight = false;

int servo_pins[4] = { 2,3,4,5 };
Servos servos(servo_pins);

MPU mpu = MPU();

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	Serial.println("Starting MPU");
	mpu.start();
	Serial.println("Waiting for data to stabilize");
	if (!wait_for_MPU_ready(mpu)) {
		Serial.println("Startup failed.");
		while (1) {
			Serial.print(" 1 ");
		}
	}
	
}

// the loop function runs over and over again until power down or reset
void loop() {
	mpu.poll();
	autonomous_flight = !autonomous_flight;
	if (autonomous_flight) Serial.println(mpu.vel[0]);
}
