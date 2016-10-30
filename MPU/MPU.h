#pragma


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
class MPU {

	float sample[100];
	unsigned long int sample_pos = 0;
	float average;
	float stddev;
	int valid_samples;

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

	/*Tries to initialize the MPU*/
	int start();

	//Gets all current data from the MPU's DMP. Takes at least 2.5 milliseconds to do its job
	int poll();

	/*Checks to see if the sensor is ready. It will call poll() once, and add that to a circular array
	of samples. Every hundred times it's called, it will check to see if the standard deviation of the 
	data is below a threshold. When that happens it increments valid_samples (because sometimes the data
	will stay stable for a moment without staying stable). When valid_samples hits five, the sensor is
	considered ready.*/
	bool get_sensor_ready();

	float accel_x() { return accel[0]; }
	float accel_y() { return accel[1]; }
	float accel_z() { return accel[2]; }
	float gyro_y() { return ypr[0]; }
	float gyro_p() { return ypr[1]; }
	float gyro_r() { return ypr[2]; }
};

/*Just repeatedly calls get_sensor_ready until it returns true. If more than 45 seconds passes
without the mpu being ready, it times out. It typically takes 20 to 30 seconds for the data
coming from the MPU to stabilize.*/
bool wait_for_MPU_ready(MPU& mpu);