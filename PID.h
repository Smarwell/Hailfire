#pragma once 

class PID
{
private:
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

public:
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


	double proc(double processVariable)
	{
		frame = micros()/1000.0 - frame;
		previousProccessVariables[counter%5]=processVariable;
		currentError=setPoint-processVariable;
		integral+=currentError*frame;
		derivative=(currentError-previousError)/frame;
		output=(constantP*currentError)+(constantI*integral)+(constantD*derivative);
		previousError=currentError;
		counter++;
		return output;
	}

};