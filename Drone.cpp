
#include "Drone.h"

bool Drone::init() {
	bool ready = true;
	servos.start();
	mpu.start();
	wait_for_MPU_ready(mpu);

}