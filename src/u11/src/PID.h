#ifndef PID_H_
#define PID_H_

#include <vector>

class PID {
public:
	PID(double _kp, double _ki, int _memi, double _kd);
	const double control(double error);

private:
	const double kp, ki, kd;
	int memi;
	double sum;
	std::vector<double> mem;
};

#endif /* PID_H_ */
