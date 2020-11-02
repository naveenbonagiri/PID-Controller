#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
  
    p_error = 0;
    i_error = 0;
    d_error = 0;

    prev_cte = 0;
  
  	//init_done = false;
}

void PID::UpdateError(double cte) 
{
	p_error = cte;

	d_error = cte - prev_cte;
	prev_cte = cte;

	i_error += cte;
}

double PID::TotalError() 
{
	double total_error = p_error * Kp + d_error * Kd + i_error * Ki;	
	return total_error;
}