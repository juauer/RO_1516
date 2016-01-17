#include <ros/ros.h>
#include <tf/tf.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
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

ros::Subscriber sub_odo;
ros::Publisher pub_y;
ros::Publisher pub_error;
ros::Publisher pub_pidout;
ros::Publisher pub_drive;
double y     = 0.0;
double error = 0.0;

void odoCB(const nav_msgs::Odometry &msg) {
	double uyaw = tf::getYaw(msg.pose.pose.orientation);
	double syaw = uyaw <= M_PI ? uyaw : uyaw - 2.0 * M_PI;
	y     = msg.pose.pose.position.y;
	error = Controller::ty == y ? 0
			: atan2(Controller::ty - y, Controller::ahead) - syaw;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controller");
	ros::Time::init();
	ros::Rate r(Controller::rate);
	ros::NodeHandle n;
	sub_odo    = n.subscribe("/ackermann_vehicle/odom", 100, odoCB);
	pub_y      = n.advertise<std_msgs::Float64>("/ackermann_vehicle/y", 100);
	pub_error  = n.advertise<std_msgs::Float64>("/ackermann_vehicle/error", 100);
	pub_pidout = n.advertise<std_msgs::Float64>("/ackermann_vehicle/pidout", 100);
	pub_drive  = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 100);
	std_msgs::Float64 msg_plot;
	ackermann_msgs::AckermannDriveStamped msg_drive;
	msg_drive.drive.jerk                    = 0;
	msg_drive.drive.steering_angle_velocity = Controller::steer_vel;
	msg_drive.drive.speed                   = Controller::speed;
	msg_drive.drive.acceleration            = Controller::accel;
	PID pid(1, 0, 1, 0);
	ros::Duration(5).sleep();

	for(int i=0; i<Controller::its; ++i) {
		const double pidout = pid.control(error);
		msg_plot.data = y;
		pub_y.publish(msg_plot);
		msg_plot.data = error;
		pub_error.publish(msg_plot);
		msg_plot.data = pidout;
		pub_pidout.publish(msg_plot);
		msg_drive.header.stamp         = ros::Time(0);
		msg_drive.drive.steering_angle = pidout;
		pub_drive.publish(msg_drive);
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
