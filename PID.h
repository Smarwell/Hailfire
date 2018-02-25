#pragma once 

#include "MPU\MPU6050_6Axis_MotionApps20_edited.h"

const int der_sample_size = 3;

class PID {
public:
	float pgain;	//proportional gain - dictates the relationship between the error and response
	float igain;	//integral gain - dictates the relationship between the integral of the error and the response
	float dgain;	//differential gain - dictates the relationship between the derivative of the error and the response

	float setpoint;
	float error;
	float past_error;
	float past_slopes[der_sample_size];
	int slope;
	float integ_error; //The time integral of the error
	float der_error; //The time derivative of the error
	float val;	//The most recent value plugged in
	float result;	//The most recent result

	unsigned long last_update; //holds the time of the last update
	unsigned int frame; //holds the time since the last update

	PID(float p, float i, float d) {
		pgain = p;
		igain = i;
		dgain = d;
		setpoint = 0.0;
		error = 0.0;
		past_error = 0.0;
		slope = 0;
		for (int i = 0; i < der_sample_size; i++) {
			past_slopes[i] = 0.0;
		}
		last_update = micros();
	}

	void set_setpoint(float new_setpoint) {
		setpoint = new_setpoint;
	}

	void reset() {
		integ_error = 0.0;
		error = 0.0;
		past_error = 0.0;
		slope = 0;
		for (int i = 0; i < der_sample_size; i++) {
			past_slopes[i] = 0.0;
		}
		last_update = micros();
	}

	inline float integral_error() {
		integ_error = integ_error + error*frame / 1000.0;
		return integ_error;
	}

	inline float derivative_error() {
		past_slopes[slope % der_sample_size] = frame ? (error - past_error) / frame : 0;
		slope++;
		der_error = 0;
		for (int i = 0; i < der_sample_size; i++) {
			der_error = der_error + past_slopes[i] / der_sample_size;
		}
		return der_error;
	}

	float calc(float current_val) {
		val = current_val;
		error = FCOMP_PI(setpoint, current_val);
		frame = (micros() - last_update);
		last_update = micros();
		result = pgain*error + igain*integral_error() + dgain*derivative_error();
		past_error = error;
		return result;
	}

	void report() {
		Serial1.println("\n\n\n" + String(setpoint) + "\t" + String(val) + "\t" + String(error));
	}
};