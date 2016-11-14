
#include "Drone.h"

void Drone::init() {
	servos.start();
	mpu.start();
	ready = wait_for_MPU_ready(mpu);

	ready = ready && comm_check();
	//put a motor startup sequence in here

	flight_mode = LANDED;
}

void Drone::set_mode(uint8_t mode) {
	flight_mode = mode;
}

void Drone::set_thrust(uint8_t arg) {
	servos.set_power(0, arg / 255.0f);
	servos.set_power(1, arg / 255.0f);
	servos.set_power(2, arg / 255.0f);
	servos.set_power(3, arg / 255.0f);

}

void Drone::update() {
	//check comms, read mpu data, and update pid controllers here.
	check_for_message();
	mpu.poll();

}

void Drone::periodic_update() {
	//check battery voltage here, and whatever else gets thought of.
}

