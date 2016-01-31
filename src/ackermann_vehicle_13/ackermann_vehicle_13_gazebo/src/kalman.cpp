#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>

ros::Subscriber sub_laser;
ros::Publisher pub_x;
ros::Publisher pub_v;
std_msgs::Float64 msg_kalman;
Eigen::Vector2f x;
Eigen::Matrix2f p;
Eigen::Matrix2f a;
Eigen::Matrix2f h;
Eigen::Matrix2f q;
Eigen::Matrix2f r;
int t = 0;

void predict(Eigen::Vector2f &x2, Eigen::Matrix2f &p2) {
	x2 << a * x;
	p2 << a*p*a.transpose() + q;
}

void update(Eigen::Vector2f m, Eigen::Vector2f x2, Eigen::Matrix2f p2, Eigen::Vector2f &x3, Eigen::Matrix2f &p3) {
	Eigen::Matrix2f i = h * p2 * h.transpose();
	Eigen::Matrix2f k = p2 * h.transpose() * i.inverse();
	x3 << x2 + k*(m - h*x2);
	p3 << (Eigen::Matrix2f::Identity() - k*h) * p2;
}

void laserCB(const sensor_msgs::LaserScan &msg) {
	Eigen::Vector2f m, x2, x3;
	Eigen::Matrix2f p2, p3;
	int time = msg.header.stamp.nsec;
	m(0) = msg.ranges[269] + 0.5;
	m(1) = 1000000000 * (m(0) - x(0, 0)) / (time - t);
	predict(x2, p2);
	update(m, x2, p2, x3, p3);
	t = time;
	x << x3;
	p << p3;
	msg_kalman.data = x(0);
	pub_x.publish(msg_kalman);
	msg_kalman.data = x(1);
	pub_v.publish(msg_kalman);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kalman");
	ros::Time::init();
	ros::Rate rr(10);
	ros::NodeHandle n;
	x << 0,
	     0;
	p << 1000, 0,
	     0,    1000;
	a << 1, 0.04,
	     0, 1;
	h << 1, 0,
	     0, 1;
	q << 1, 0,
	     0, 1;
	r << 1, 0,
	     0, 1;
	sub_laser = n.subscribe("/ackerman_vehicle/laser/scan", 10, laserCB);
	pub_x     = n.advertise<std_msgs::Float64>("/kalman_prediction_x", 10);
	pub_v     = n.advertise<std_msgs::Float64>("/kalman_prediction_v", 10);
	ros::Duration(10).sleep();

	while(ros::ok()) {
		ros::spinOnce();
		rr.sleep();
	}

	return 0;
}
