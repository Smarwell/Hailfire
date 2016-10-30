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
	mpu.start();
	while (!mpu.get_sensor_ready());
}

// the loop function runs over and over again until power down or reset
void loop() {
	/*
	for (int i = 0; i < 0; i++) {
		mpu.poll();
		delay(0);
	}*/
	mpu.poll();
	Serial.println(String(mpu.accel_x()));
	//Serial.println("\n\n");
}
