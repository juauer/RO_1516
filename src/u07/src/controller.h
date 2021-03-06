#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <nav_msgs/Odometry.h>

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

namespace Controller {
	const int its          = 500;  // controll cycles total
	const float rate       = 10.0; // controll cycles per sec
	const double ahead     =  4.0; // look-ahead-dist to compute error on angle
	const double speed     = 10.0; // target speed
	const double accel     =  2.0; // acceleration
	const double steer_vel =  1.0; // steering velocity
	const double ty        = -0.5; // target y (0: middle of the street)
};

#endif // CONTROLLER_H
