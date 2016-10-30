
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


#include "MPU.h"
#include "Comm.h"	//allows for use of error messages
#include "Drone.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20_edited.h"	//here be dragons

#include "Wire.h"

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}



/*Tries to initialize the MPU*/
int MPU::start() {
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	mpu.initialize();

	devStatus = mpu.dmpInitialize();
	if (devStatus == 0) {
		mpu.setDMPEnabled(true);

		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

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
			fifoCount -= packetSize;
		}
		mpu.dmpGetQuaternion(&q, fifoBuffer);	//get various vectors
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);	//load the data into more easily used formats
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		accel[0] = aaReal.x;
		accel[1] = aaReal.y;
		accel[2] = aaReal.z;
	}
	return 0;
};



/*Checks to see if the sensor is ready. It will call poll() once, and add that to a circular array
of samples. Every hundred times it's called, it will check to see if the standard deviation of the
data is below a threshold. When that happens it increments valid_samples (because sometimes the data
will stay stable for a moment without staying stable). When valid_samples hits five, the sensor is
considered ready.*/
bool MPU::get_sensor_ready() {
	poll();
	sample[sample_pos % 100] = accel[0];
	sample_pos++;
	if (sample_pos < 100) {
		return false;
	}
	if (sample_pos % 100 == 0) {
		average = 0;
		for (int i = 0; i < 100; i++) {	//Calculate the average of the samples
			average += sample[i] / 100;
		}
		stddev = 0;
		for (int i = 0; i < 100; i++) {	//Calculate the standard deviation of the samples
			stddev += (sample[i] - average)*(sample[i] - average) / 100;
		}
		if (stddev < 50) {		//Is the standard deviation below 50?
			valid_samples++;
			sample_pos = 0;
		}
		return valid_samples == 5;	//only return true if 5 valid sets of samples have been taken
	}
	else {
		return false;
	}

}



/*Just repeatedly calls get_sensor_ready until it returns true. If more than 45 seconds passes
without the mpu being ready, it times out. It typically takes 20 to 30 seconds for the data
coming from the MPU to stabilize.*/
bool wait_for_MPU_ready(MPU& mpu) {
	unsigned long int start = millis();
	while (!mpu.get_sensor_ready()) {
		if (millis() - start > 45000) return false;
	}
	return true;
}