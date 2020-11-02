#include "PID.h"

#define TWIDDLE_STATE1 0x01
#define TWIDDLE_STATE2 0x02

class Twiddle {
private:
	std::vector<double> p{3};
	std::vector<double> dp{3};

	double average_error{};
	double error{};
	double best_error{};

	long int num_steps_counter{};
	int param_index{ 0 };
	void UpdateParamIndex();

	unsigned char twiddle_next_state;
public:
	void InitializeTwiddleAlgo(void);
	void ExecuteTwiddlealgo(PID& UpdateParamIndex);

	void CalculateAverageError();
	void CalcCumulativeError(double cte);

	double PerformSumDp(int num_params, std::vector<double> dp);

	void MaintainNumOfStepsCounter();

	bool passed_max_iterations{ false };
};