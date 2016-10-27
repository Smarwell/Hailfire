#ifndef PID_H
#define PID_H

using namespace std;
class PID
{
private:
	double error_value;
	double output;
	double set_point;
	double process_value;
	double proportional_gain;
	double integral_gain;
	double derivative_gain;
	double time; //this variable is a place holder for the system variable in arduino

public:
	/*PID(error_value,control_value,propotional_value,integral_value,derivative_value)
	{

	}*/
	PID()
	{
		error_value=0.0;
		output=0.0;
		set_point=0.0;
		process_value=0.0;
		proportional_gain=0.0;
		integral_gain=0.0;
		derivative_gain=0.0;
		time=0.0;
	}
	double getProportional (){return set_point-process_value;}
	double getIntegral (){return integral+error_value*time;}
	double getDerivative (){return error-previousError)/time;}
	void setoutput(double output){this.output=output;} 

	double pid()
	{
		//should we onlycalculate integral if our error value massive?
		double output=0;
		double previous_error=0;
		double integral=0;
		//while(1)
		//{
			output=proportional_gain*getProportion()+integral_gain*getIntegral()+derivative_gain*getDerivative();
			previous_error=error;
			setoutput(output);
			return output;
		//}
	}
	



}

#endif