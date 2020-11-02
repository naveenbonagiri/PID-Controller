#include <iostream>
#include <vector>
#include "twiddle.h"

// routine to perform delta parameters sum
double Twiddle::PerformSumDp(int num_params, std::vector<double> dp) 
{
	double sum_dp = 0;
	for (int index = 0; index < num_params; index++) 
	{
		sum_dp += dp[index];
	}
	return sum_dp;
}

// Increment the counter for each iteration
void Twiddle::MaintainNumOfStepsCounter() 
{
	num_steps_counter++;
}

// maintain cumulative error after initial iterations are complete
void Twiddle::CalcCumulativeError(double cte) 
{
	if (num_steps_counter > 100) 
	{
		error += cte * cte;
	}
}

// start calculating avaerage error once max iterations are finished 
void Twiddle::CalculateAverageError(void) 
{
	if (num_steps_counter > 1500) 
	{
		average_error = error / (num_steps_counter - 100);
		passed_max_iterations = true;
	}
}

void Twiddle::InitializeTwiddleAlgo(void)
{
	// begin with best error same as average error
	// increment p by dp and set the twiddle next state to increment
	best_error = average_error;
	p[param_index]  += dp[param_index];
	twiddle_next_state = TWIDDLE_STATE1;	
}
// Twiddle algorithm for tuning parameters automatically
void Twiddle::ExecuteTwiddlealgo(PID &steering_angle_pid) 
{
	p = {steering_angle_pid.Kp, steering_angle_pid.Ki, steering_angle_pid.Kd};
	dp = {0.05, 0.001, 0.05};
	double threshold = 0.00001;	
	int num_params = 3;

	while (PerformSumDp(num_params, dp) > threshold) 
	{
		switch (twiddle_next_state) 
		{
		case TWIDDLE_STATE1:
			// Update best error based on the average error 
			// increment the dp if average error is better 
			// otherwise decrement p by dp with some factor
			if (average_error < best_error) 
			{
				best_error = average_error;
				dp[param_index] *= 1.1;
              	UpdateParamIndex();
				p[param_index]  += dp[param_index];
			}
			else 
			{
				p[param_index] -= 2 * dp[param_index];
				twiddle_next_state = TWIDDLE_STATE2;
			}
			UpdateParamIndex();		
			break;
		case TWIDDLE_STATE2:
			// Update best error based on the average error 
			// increment the dp if average error is better 
			// otherwise decrement dp with some factor		
			if (average_error < best_error) 
			{
				best_error = average_error;
				dp[param_index] *= 1.1;
			} 
			else 
			{
			    p[param_index] += dp[param_index];
				dp[param_index] *= 0.9;
			}

			UpdateParamIndex();
			p[param_index] += dp[param_index];

			// change the twiddle state
			twiddle_next_state = TWIDDLE_STATE1;
			break;
		default:
			break;
		}
	}

	// update coefficients with tuned parameters
	steering_angle_pid.Kp = p[0];
	steering_angle_pid.Ki = p[1];
	steering_angle_pid.Kd = p[2];
}

// routine to maintain the paramater index
void Twiddle::UpdateParamIndex() 
{
	param_index = (param_index + 1) % p.size();
}




