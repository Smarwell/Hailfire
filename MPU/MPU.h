#pragma

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20_edited.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

class MPU {

	float sample[100];
	unsigned long int sample_pos = 0;

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
	float accel[3];
	float ypr[3];

	MPU() {};

	void start() {
		#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
			Wire.begin();
			TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
		#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
			Fastwire::setup(400, true);
		#endif
		mpu.initialize();

		devStatus = mpu.dmpInitialize();
		if (devStatus == 0) {
			mpu.setDMPEnabled(true);

			attachInterrupt(0, dmpDataReady, RISING);
			mpuIntStatus = mpu.getIntStatus();

			dmpReady = true;
			packetSize = mpu.dmpGetFIFOPacketSize();
		}
		else {
			Serial.println("MPU initialization failed");
		}
	};

	void poll() {
		if (!dmpReady) {
			return;
		}

		while (!mpuInterrupt && fifoCount < packetSize) {
			fifoCount = mpu.getFIFOCount();
		}

		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount();

		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			mpu.resetFIFO();
			Serial.println("MPU FIFO has overflowed");
		}
		else if (mpuIntStatus & 0x02) {
			while (fifoCount < packetSize) {
				fifoCount = mpu.getFIFOCount();
			}
			while (fifoCount >= packetSize) {
				mpu.getFIFOBytes(fifoBuffer, packetSize);
				fifoCount -= packetSize;
			}
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			accel[0] = aaReal.x;
			accel[1] = aaReal.y;
			accel[2] = aaReal.z;
		}
	};

	bool get_sensor_ready() {
		poll();
		sample[sample_pos%100] = accel[0];
		sample_pos++;
		if (sample_pos < 100) {
			return false;
		}
		if (sample_pos%100==0) {
			float average = 0;
			for (int i = 0; i < 100; i++) {
				average += sample[i] / 100;
			}
			float stddev = 0;
			for (int i = 0; i < 100; i++) {
				stddev += (sample[i] - average)*(sample[i] - average) / 100;
			}
			return stddev < 50;
		}
		else {
			return false;
		}
		
	}

	float accel_x() { return accel[0]; }
	float accel_y() { return accel[1]; }
	float accel_z() { return accel[2]; }
	float gyro_y() { return ypr[0]; }
	float gyro_p() { return ypr[1]; }
	float gyro_r() { return ypr[2]; }
};