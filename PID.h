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
	double dirivative;
	double integral;
	double previousError;
	double output;
	int counter;
	double previousProccessVariables[]; 

public:
	PID(double p, double i, double d, double sp, double pv, double err, double t)
	{
		constantP=p;
		constantI=i;
		constantD=d;
		setPoint=sp;
		processVariable=pv;
		currentError=err;
		dirivative=0;
		integral=0;
		previousError=0;
		output=0;
		counter=1;
		previousProccessVariables[0]=pv;
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


	double proc(double time, double processVariable)
	{
		previousProccessVariables[counter%5]=processVariable;
		currentError=setPoint-processVariable;
		integral+=currentError*time;
		dirivative=(currentError-previousError)/time;
		output=(constantP*currentError)+(constantI*integral)+(constantD*dirivative);
		previousError=currentError;
		counter++;
		return output;
	}

};