#pragma once

#include "Drone.h"

void Drone::init() {
	servos.start();
	ready = true;
	mpu.start();
	ready = wait_for_MPU_ready(mpu);

	ready = ready && comm_check();

	flight_mode = LANDED;

	reset_gyro_controllers();
	
}

void Drone::set_mode(uint8_t mode) {
	flight_mode = mode;
}

void Drone::set_thrust(uint8_t arg) {
	servos.set_baseline(arg / 255.0f);
}

void Drone::update() {
	check_for_message();
	mpu.poll();
	servos.reset_diffs();
	res = yaw_pid.calc(mpu.gyro_y());
	servos.manip_motors(0, 3, 1, 2, res);
	servos.manip_motors(0, 1, 2, 3, pitch_pid.calc(mpu.gyro_p()));
	servos.manip_motors(1, 3, 0, 2, roll_pid.calc(mpu.gyro_r()));
}

void Drone::periodic_update() {
	//check battery voltage here, and whatever else gets thought of.
}

void Drone::reset_gyro_controllers() {
	mpu.poll();
	yaw_pid.set_setpoint(mpu.ypr[0]);
	pitch_pid.set_setpoint(mpu.ypr[1]);
	roll_pid.set_setpoint(mpu.ypr[2]);
	yaw_pid.reset();
	pitch_pid.reset();
	roll_pid.reset();
}

void Drone::reset_gyro() {
	//mpu.gyro_offsets[0] = mpu.ypr[0];
	//mpu.gyro_offsets[1] = mpu.ypr[1];
	//mpu.gyro_offsets[2] = mpu.ypr[2];
}