#pragma once

#include "Drone.h"

void Drone::init() {
	servos.start();
	//mpu.start();
	//ready = wait_for_MPU_ready(mpu);

	//ready = ready && comm_check();

	flight_mode = LANDED;

	reset_gyro_setpoints();
}

void Drone::set_mode(uint8_t mode) {
	flight_mode = mode;
}

void Drone::set_thrust(uint8_t arg) {
	servos.set_baseline(arg / 255.0f);
}

void Drone::update() {
	check_for_message();
	//mpu.poll();
	//servos.manip_motors(0, 3, 1, 2, yaw_controller.proc(mpu.gyro_y()));
	//servos.manip_motors(0, 1, 2, 3, pitch_controller.proc(mpu.gyro_p()));
	//servos.manip_motors(1, 3, 0, 2, roll_controller.proc(mpu.gyro_r()));
}

void Drone::periodic_update() {
	//check battery voltage here, and whatever else gets thought of.
}

void Drone::reset_gyro_setpoints() {
	yaw_controller.setSetPoint(mpu.gyro_offsets[0]);
	pitch_controller.setSetPoint(mpu.gyro_offsets[1]);
	roll_controller.setSetPoint(mpu.gyro_offsets[2]);
}

void Drone::reset_gyro() {
	mpu.gyro_offsets[0] = mpu.ypr[0];
	mpu.gyro_offsets[1] = mpu.ypr[1];
	mpu.gyro_offsets[2] = mpu.ypr[2];
}