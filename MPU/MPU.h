#pragma once

#include "../Comm.h"	//allows for use of error messages
//#include "../Drone.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20_edited.h"	//here be dragons
#include "Wire.h"


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}


/*
Acts as a wrapper for the MPU6050 class

MPU == motion processor unit
DMP == digital motion processor (special processor built into the MPU)
FIFO == First in first out (a 1024 byte buffer in the MPU)

To use, simply create an object, call start(), and then call wait_for_MPU_ready().
Once that finishes (assuming it was successful), call poll() to get current data from
the MPU. The DMP is set to write its data to the FIFO at 100 Hz. If poll() is called fewer
than ten milliseconds after the last time poll() was called, there will not be any data
in the FIFO yet, and it will wait until there is data to retrieve it and return. The
minimum run time for poll() is about 2.5 milliseconds.

If poll() is not called for long enough, the FIFO can overflow. In this case the FIFO is
simply reset, though all of the object's data won't be updated until the next time poll()
is called.

For typically 20 to 30 seconds after the MPU is started up, the data will drift
quite a bit. For this reason, the drone should wait for the data to stabilize before trying
to fly. Once the 'warmup' period has passed, the gyroscope data is in general very reliable,
but accelerometer data is pretty noisy.
*/

bool l = true;

class MPU {

	unsigned long int sample_pos = 0;

	bool calibrated = false;

	unsigned long int start_t;
	unsigned long int end_t;

	MPU6050 mpu;

	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector

public:
	float accel_offsets[3];
	float gyro_offsets[3];

	float* old_accel;
	float accel[3];
	float ypr[3];
	float vel[3];

	float telemetry[9];

	MPU() {};

	/*Tries to initialize the MPU*/
	bool start() {

		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
		mpu.initialize();

		devStatus = mpu.dmpInitialize();
		if (devStatus == 0) {
			mpu.setDMPEnabled(true);

			//attachInterrupt(0, dmpDataReady, RISING);
			//mpuIntStatus = mpu.getIntStatus();

			calibrated = false;

			dmpReady = true;
			packetSize = mpu.dmpGetFIFOPacketSize();
			Serial1.println("MPU started up successfully");
			return true;
		}
		else {
			Serial1.println("MPU failed to start up");
			return false;
		}
	};

	//Gets all current data from the MPU's DMP. Takes at least 2.5 milliseconds to do its job,
	//and will wait for 10 milliseconds to have passed since the last call if it is called
	//in fewer than 10 milliseconds from the last call (A side effect of the DMP's functionality.)
	int poll() {

		if (!dmpReady) {
			return 1;
		}
		fifoCount = mpu.getFIFOCount();
		while (fifoCount < packetSize) {	//wait for the correct packet size
			fifoCount = mpu.getFIFOCount();
		}
		if (fifoCount == 1024) {	//check for FIFO overflow
			mpu.resetFIFO();
			send_message(MPU_FIFO_OVERFLOW, "The MPU's FIFO has overflowed and been reset");
			return 2;
		}
		else if (mpuIntStatus & 0x02 || true) {		//data ready
			
			/*while (fifoCount >= packetSize) {	//If multiple packets have been written to the FIFO,
				mpu.getFIFOBytes(fifoBuffer, packetSize);	//Read packets until the most recent one is reached
				time_passed = micros() - frame_start;
				frame_start = micros();
				fifoCount -= packetSize;
			}*/
			mpu.getFIFOBytes(fifoBuffer, packetSize);	//Read packets until the most recent one is reached
			mpu.resetFIFO();

			mpu.dmpGetQuaternion(&q, fifoBuffer);	//get various vectors
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);	//load the data into more easily used formats
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			//old_accel = accel;
			accel[0] = aaReal.x;
			accel[1] = aaReal.y;
			accel[2] = aaReal.z;
		}
		return 0;
	};

	int calibrate() {
		calibrated = false;
		gyro_offsets[0] = 0.0;
		gyro_offsets[1] = 0.0;
		gyro_offsets[2] = 0.0;

		for (int i = 0; i < 100; i++) {
			if (poll() == 1) {
				send_message(MPU_INIT_FAILED, "MPU did not initialize properly");
				return 1;
			}
			accel_offsets[0] += accel[0] / 100.0;	//putting this in a for loop breaks it HARD.
			accel_offsets[1] += accel[1] / 100.0;	//Undefined behavior for seemingly no reason...
			accel_offsets[2] += accel[2] / 100.0;
			gyro_offsets[0] = gyro_offsets[0] + ypr[0];
			gyro_offsets[1] = gyro_offsets[1] + ypr[1];
			gyro_offsets[2] = gyro_offsets[2] + ypr[2];
			//Serial1.println("Calibrate: " + String(ypr[0]));
		}
		gyro_offsets[0] = gyro_offsets[0] / 100.0;
		gyro_offsets[1] = gyro_offsets[1] / 100.0;
		gyro_offsets[2] = gyro_offsets[2] / 100.0;
		//determined experimentally
		/*
		accel_offsets[0] = 22.56;
		accel_offsets[1] = -14.82;
		accel_offsets[2] = -318.353;
		*/

		vel[0] = 0;
		vel[1] = 0;
		vel[2] = 0;
		calibrated = true;
		poll();
		return 0;
		Serial1.println("MPU calibrated");
	}

	float* get_telemetry() {
		memcpy(telemetry, ypr, 3);
		memcpy(telemetry + 3, accel, 3);
		memcpy(telemetry + 6, vel, 3);
		return telemetry;
	}

	void reset() {
		mpu.resetFIFO();
	}

	float accel_x() { return accel[0]; }
	float accel_y() { return accel[1]; }
	float accel_z() { return accel[2]; }
	float gyro_y() { return ypr[0]; }
	float gyro_p() { return ypr[1]; }
	float gyro_r() { return ypr[2]; }
};

bool wait_for_MPU_ready(MPU& mpu) {
	Serial1.println("Waiting for gyroscope measurements to stabilize...");
	unsigned long int start = millis();
	while (millis() - start < 30000) {
		if (mpu.poll() == 1) {
			send_message(MPU_INIT_FAILED, "MPU did not initialize properly");
			return false;
		}
	}
	mpu.calibrate();	//if mpu.poll() worked properly, so will this
	return true;
}