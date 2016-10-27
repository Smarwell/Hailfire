#pragma once

//For the Hailfire bot
//Author Jacob Sacco


#include <Wire.h>
#include <Arduino.h>

#define MPU_ADDR 0b1101000	//make sure the address pin has been pulled to ground

#define PWR_MGMT_1 0x6b
#define PWR_MGMT_2 0x6c
#define XG_OFFSET_H  0x13
#define XG_OFFSET_L  0x14
#define YG_OFFSET_H  0x15
#define YG_OFFSET_L  0x16
#define ZG_OFFSET_H  0x17
#define ZG_OFFSET_L  0x18
#define USER_CTRL 0x6a

#define DATA_ADDR 0x3b

#define STARTUP_PWR 0b00000001	//Sets the clock to use the X axis gyroscope as a reference
#define STARTUP_CTRL 0b10000000	//Turns on the DMP

const double gyro_x_offset = 382.7;
const double gyro_y_offset = -152.0;
const double gyro_z_offset = 41.3;


class MPU6050 {
	int AcX;
	int AcY;
	int AcZ;
	int temp;
	int GyX;
	int GyY;
	int GyZ;
	unsigned long int last_poll;
	double dt;

	double GyroX;
	double GyroY;
	double GyroZ;

public:

	MPU6050() {
	};

	void start() {
		Wire.begin();
		write_byte(PWR_MGMT_1, STARTUP_PWR);
		write_byte(USER_CTRL, STARTUP_CTRL);
		last_poll = micros();
		poll();
	}

	void write_byte(uint8_t reg, uint8_t data) {
		Wire.beginTransmission(MPU_ADDR);
		Wire.write(reg);
		Wire.write(data);
		Wire.endTransmission(true);
	}

	void write_word(int16_t reg, int16_t data) {
		Wire.beginTransmission(MPU_ADDR);
		Wire.write(reg);
		Wire.write(data >> 8);
		Wire.write(data);
		Wire.endTransmission(true);
	}

	void poll() {
		Wire.beginTransmission(MPU_ADDR);
		Wire.write(DATA_ADDR);
		Wire.endTransmission(false);
		Wire.requestFrom(MPU_ADDR, 14);
		AcX = Wire.read() << 8 | Wire.read();
		AcY = Wire.read() << 8 | Wire.read();
		AcZ = Wire.read() << 8 | Wire.read();
		temp = Wire.read() << 8 | Wire.read();
		GyX = Wire.read() << 8 | Wire.read();
		GyY = Wire.read() << 8 | Wire.read();
		GyZ = Wire.read() << 8 | Wire.read();

		dt = (micros() - last_poll)/1000000.0;
		last_poll = micros();

		GyroX += (GyX + gyro_x_offset)*dt;
		GyroY += (GyY + gyro_y_offset)*dt;
		GyroZ += (GyZ + gyro_z_offset)*dt;
		
	}

	int get_accel_x() { return AcX; }
	int get_accel_y() { return AcY; }
	int get_accel_z() { return AcZ; }
	int get_temp() { return temp; }
	double get_gyro_x() { return GyroX; }
	double get_gyro_y() { return GyroY; }
	double get_gyro_z() { return GyroZ; }
};





