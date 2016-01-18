#include "PID.h"

PID::PID(double _kp, double _ki, int _memi, double _kd):
		kp(_kp),
		ki(_ki),
		memi(std::max(1, _memi)),
		kd(_kd),
		sum(0.0) {
	mem.push_back(0.0);
}

const double PID::control(double error) {
	const double asc = error - mem.back();
	sum += error;
	mem.push_back(error);

	if(mem.size() > memi) {
		sum -= mem.front();
		mem.erase(mem.begin());
	}

	return kp*error + ki*sum + kd*asc;
}
