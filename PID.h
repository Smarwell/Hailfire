#pragma once 
/*
class PID
{
private:
public:
	double constantP;
	double constantI;
	double constantD;
	double setPoint;
	double processVariable;
	double currentError;
	double derivative;
	double integral;
	double previousError;
	double output;
	int counter;
	double previousProccessVariables[5]; 
	long unsigned int frame;

	PID(double p, double i, double d, double sp = 0.0 , double pv = 0.0, double err = 0.0)
	{
		constantP=p;
		constantI=i;
		constantD=d;
		setPoint=sp;
		processVariable=pv;
		currentError=err;
		derivative=0;
		integral=0;
		previousError=0;
		output=0;
		counter=1;
		previousProccessVariables[0]=pv;
		frame = micros()/1000.0;
	}

	void setSetPoint(double point){setPoint=point;}
	double getSetPoint(){return setPoint;}
	double* getPreviousProccessVariables(){return previousProccessVariables;}
	void setProccessVariable(double pv)
	{
		processVariable=pv;
		previousProccessVariables[counter%5]=pv;
		counter++;
	}


	float proc(float processVariable)
	{
		frame = micros()/1000.0 - frame;
		previousProccessVariables[counter%5]=processVariable;
		currentError=setPoint-processVariable;
		integral+=currentError*frame;
		derivative=(currentError-previousError)/frame;
		Serial1.println(String(currentError) + "    " + String(constantP));
		delay(50);
		output=(constantP*currentError)+(constantI*integral)+(constantD*derivative);
		previousError=currentError;
		counter++;
		return output;
	}

};

*/

class PID {
public:
	float pgain;	//proportional gain - dictates the relationship between the error and response
	float igain;	//integral gain - dictates the relationship between the integral of the error and the response
	float dgain;	//differential gain - dictates the relationship between the derivative of the error and the response

	float setpoint;
	float error;
	float past_error;
	float integ_error; //The time integral of the error
	float der_error; //The time derivative of the error
	float val;	//The most recent value plugged in
	float res;	//The most recent result

	unsigned long last_update; //holds the time of the last update
	unsigned int frame; //holds the time since the last update

	PID(float p, float i, float d) {
		pgain = p;
		igain = i;
		dgain = d;
		setpoint = 0.0;
		error = 0.0;
		past_error = 0.0;
		last_update = micros();
	}

	void set_setpoint(float new_setpoint) {
		setpoint = new_setpoint;
	}

	void reset() {
		integ_error = 0.0;
		error = 0.0;
		past_error = 0.0;
		last_update = micros();
	}

	float calc(float current_val) {
		val = current_val;
		error = setpoint - current_val;
		frame = (micros() - last_update);
		last_update = micros();
		integ_error = integ_error + error*frame / 1000.0;
		der_error = (error - past_error) / frame;
		res = pgain*error + igain*integ_error + dgain*der_error;
		past_error = error;
		return res;
	}
};