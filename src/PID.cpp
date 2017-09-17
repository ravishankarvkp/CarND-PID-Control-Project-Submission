#include "PID.h"
#include <cmath>
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{

	/* Initialize - PID parameter constants */
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	/* Reset PID error coefficients to 0. */
	this->p_error = 0.0;
	this->i_error = 0.0;
	this->d_error = 0.0;

	// Training and evaluation variables
	this->i_e_fabs_error_ = 0;
	epochCumulativeError_ = 0;
	previousEpochError_ = 0;
	currentEpochError_ = 0;
	needsTraining_ = true;
	counter_ = 0;
}

double PID::getKp() const { return Kp; }
double PID::getKi() const { return Ki; }
double PID::getKd() const { return Kd; }

double PID::TotalError()
{
	return -Kp * p_error - Ki * i_error - Kd * d_error;
}

void PID::UpdateError(double cte)
{
	d_error = (cte - p_error); /* Note, this would eval incorrectly for very 1st frame. */
	p_error = cte; 
	i_error += (cte);

	/* Keep track of the epoch error */
	i_e_fabs_error_ += fabs(cte); /* Used to calculate the Ki derivative */
	epochCumulativeError_ += pow(cte, 2);
	
	counter_++;
}


/** Tune Kp, Ki and Kd parameters.
* Partial derivatives of f(Kp, Ki, Kd) = -Kp * p_error - Ki * i_error - Kd * d_error
* WITH RESPECT TO Kp, Ki, Kd are -p_error, -i_error and -d_error respectively.
* Note: For i_error, since CTE can be +ve or -ve, fabs() value is used for the purpose of determining partial derivative for Ki.
*/
void PID::backProp() 
{
	/* Determine the current epoch error. */
	currentEpochError_ = sqrt(epochCumulativeError_ / epochLength_);

	if (needsTraining_)
	{
		// Permanently stop training once criteria has been met.
		needsTraining_ = currentEpochError_ > errorThreshold_;

		// Determine the deltaEpochError
		double deltaError = previousEpochError_ - currentEpochError_;
		previousEpochError_ = currentEpochError_;

		/*
			* Backpropagate - Kx error and throttle the degree of change through learning Rate
			* Kx: Kp, Ki or Kd
			* dx: partial derivative for Kp, Ki or Kd respectively
			* dE: total delta error over the whole epoch (previous epoch - current epoch)
			*/
		Kp -= Kp * deltaError * learnRate_ * (-p_error);
		Ki -= Ki * deltaError * learnRate_ * (-i_e_fabs_error_);
		Kd -= Kd * deltaError * learnRate_ * (-d_error);
	}
}


void PID::resetEpochError() {
	i_e_fabs_error_ = 0;
	epochCumulativeError_ = 0;
}