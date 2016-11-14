
#include "MPU.h"

//Attempts to initialize the mpu
int MPU::start() {
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	mpu.initialize();

	devStatus = mpu.dmpInitialize();
	if (devStatus == 0) {
		mpu.setDMPEnabled(true);

		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		calibrated = false;

		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
		return 0;
	}
	else {
		return 1;
	}
};


//Gets all current data from the MPU's DMP. Takes at least 2.5 milliseconds to do its job,
//and will wait for 10 milliseconds to have passed since the last call if it is called
//in fewer than 10 milliseconds from the last call (A side effect of the DMP's functionality.
int MPU::poll() {

	if (!dmpReady) {
		return 1;
	}
	while (!mpuInterrupt && fifoCount < packetSize) {	//wait for the correct packet size
		fifoCount = mpu.getFIFOCount();
	}
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {	//check for FIFO overflow
		mpu.resetFIFO();
		return 2;
	}
	else if (mpuIntStatus & 0x02) {		//data ready

		while (fifoCount >= packetSize) {	//If multiple packets have been written to the FIFO,
			mpu.getFIFOBytes(fifoBuffer, packetSize);	//Read packets until the most recent one is reached
			time_passed = micros() - frame_start;
			frame_start = micros();
			fifoCount -= packetSize;
			send_message(MPU_FIFO_OVERFLOW, "The MPU's FIFO has overflowed and been reset");
		}
		mpu.dmpGetQuaternion(&q, fifoBuffer);	//get various vectors
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);	//load the data into more easily used formats
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		old_accel = accel;
		accel[0] = aaReal.x;
		accel[1] = aaReal.y;
		accel[2] = aaReal.z;

		if (calibrated) {
			for (int i = 0; i < 3; i++) {
				accel[i] -= accel_offsets[i];
				ypr[i] -= gyro_offsets[i];
				vel[i] += ((accel[i] + old_accel[i]) / 2.0 * time_passed) / 1000000.0;
			}
		}
	}
	return 0;
};

int MPU::calibrate() {
	for (int i = 0; i < 100; i++) {	//would go to 1000 normally
		if (poll() == 1) {
			send_message(MPU_INIT_FAILED, "MPU did not initialize properly");
			return 1;
		}
		accel_offsets[0] += accel[0] / 1000.0;	//putting this in a for loop breaks it HARD.
		accel_offsets[1] += accel[1] / 1000.0;	//Undefined behavior for seemingly no reason...
		accel_offsets[2] += accel[2] / 1000.0;
		gyro_offsets[0] += (ypr[0] / 1000.0);
		gyro_offsets[1] += (ypr[1] / 1000.0);
		gyro_offsets[2] += (ypr[2] / 1000.0);
	}
	//determined experimentally
	accel_offsets[0] = 22.56;
	accel_offsets[1] = -14.82;
	accel_offsets[2] = -318.353;
	gyro_offsets[0] = 20.89;
	gyro_offsets[1] = 0.77;
	gyro_offsets[2] = -0.48;

	vel[0] = 0;
	vel[1] = 0;
	vel[2] = 0;
	calibrated = true;
	return 0;
}

bool wait_for_MPU_ready(MPU& mpu) {
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