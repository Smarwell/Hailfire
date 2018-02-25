#pragma once

#include "Drone.h"

extern int update_counter;

void Drone::init() {
	Serial1.println("\n\n\nStarting up");
	servos.start();
	ready = mpu.start();
	ready = ready && wait_for_MPU_ready(mpu);

	ready = ready && comm_check();

	flight_mode = LANDED;

	reset_pid_controllers();
	
	if (ready) {
		Serial1.println("Drone is ready!");
	}
	else {
		Serial1.println("Startup failed");
	}
}

void Drone::set_mode(uint8_t mode) {
	flight_mode = mode;
}

void Drone::set_thrust(uint8_t arg) {
	if (!pid_enabled) return;

	servos.set_baseline(arg / 255.0f);
}

void Drone::update() {
	check_for_message();
	unsigned long start;
	unsigned long end;
	delay(10);
	start = micros();
	mpu.poll();
	end = micros();
	if (pid_enabled) {
		servos.reset_diffs();
		servos.manip_motors(0, 3, 1, 2, yaw_pid.calc(mpu.gyro_y()));
		servos.manip_motors(2, 3, 0, 1, pitch_pid.calc(mpu.gyro_p()));
		servos.manip_motors(1, 3, 0, 2, roll_pid.calc(mpu.gyro_r()));
		if (update_counter % 10 == 0) {
			//pitch_pid.report();
			//servos.report();
			//mpu.report();
		}
	}
	if (update_counter % 5 == 0) {
		periodic_update();
	}
	update_counter++;
}

void Drone::periodic_update() {
	//check battery voltage here, and whatever else gets thought of.
	//Serial1.println(String(servos.get_power(0)) + "   " + String(mpu.ypr[2]));
}

void Drone::reset_pid_controllers() {
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

void Drone::kill_pid_controllers() {
	set_thrust(0);
	pid_enabled = false;
	servos.reset_diffs();
	Serial1.println("PID controllers disabled");
}

void Drone::reenable_pid_controllers() {
	pid_enabled = true;
	set_thrust(0);
	mpu.reset();
	servos.reset_diffs();
	reset_pid_controllers();
	Serial1.println("PID controllers reenabled");
}