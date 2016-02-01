#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Dense>

ros::Subscriber sub_model;
ros::Subscriber sub_laser;
ros::Publisher pub_m_x;
ros::Publisher pub_m_v;
ros::Publisher pub_k_x;
ros::Publisher pub_k_v;
std_msgs::Float64 msg_kalman;
Eigen::Vector2f x, m, gt;
Eigen::Matrix2f p;
Eigen::Matrix2f a;
Eigen::Matrix2f h;
Eigen::Matrix2f q;
Eigen::Matrix2f r;
std::vector<Eigen::Vector2f> errors;
int t = 0;

void printCovarianceMatrix() {
	Eigen::Vector2f mu;
	Eigen::Matrix2f s;
	mu << 0, 0;
	s  << 0, 0, 0, 0;

	for(std::vector<Eigen::Vector2f>::const_iterator i=errors.begin(); i!=errors.end(); ++i)
		mu += *i;

	mu /= errors.size();

	for(std::vector<Eigen::Vector2f>::const_iterator i=errors.begin(); i!=errors.end(); ++i) {
		float xi =(*i)(0) - mu(0);
		float vi =(*i)(1) - mu(1);
		s(0, 0) += xi*xi;
		s(0, 1) += xi*vi;
		s(1, 0) += vi*xi;
		s(1, 1) += vi*vi;
	}

	s /= errors.size();
	ROS_INFO("SENSOR-ERROR COVARIANCE: (%.4f, %.4f; %.4f, %.4f)", s(0, 0), s(0, 1), s(1, 0), s(1, 1));
}

void predict(Eigen::Vector2f &x2, Eigen::Matrix2f &p2) {
	x2 << a * x;
	p2 << a*p*a.transpose() + q;
}

void update(Eigen::Vector2f m, Eigen::Vector2f x2, Eigen::Matrix2f p2, Eigen::Vector2f &x3, Eigen::Matrix2f &p3) {
	Eigen::Matrix2f i = h * p2 * h.transpose() + r;
	Eigen::Matrix2f k = p2 * h.transpose() * i.inverse(); // WARN: should be pinv!
	x3 << x2 + k*(m - h*x2);
	p3 << p2 - k*h*p2;
}

void laserCB(const sensor_msgs::LaserScan &msg) {
	Eigen::Vector2f x2, x3;
	Eigen::Matrix2f p2, p3;
	int time = msg.header.stamp.nsec;
	float x0 = m(0);
	m(0) = msg.ranges[269] + 0.5;
	m(1) = 1000000000 * (m(0) - x0) / (time - t);
	predict(x2, p2);
	update(m, x2, p2, x3, p3);
	ROS_INFO("1) PREDICT:\nx: (%.2f, %.2f)\np: (%.2f, %.2f, %.2f, %.2f)", x2(0), x2(1), p2(0, 0), p2(0, 1), p2(1, 0), p2(1, 1));
	ROS_INFO("2) MEASURE:\nm: (%.2f, %.2f)", m(0), m(1));
	ROS_INFO("3) UPDATE:\nx: (%.2f, %.2f)\np: (%.2f, %.2f, %.2f, %.2f)", x3(0), x3(1), p3(0, 0), p3(0, 1), p3(1, 0), p3(1, 1));
	t = time;
	x << x3;
	p << p3;
	msg_kalman.data = m(0);
	pub_m_x.publish(msg_kalman);
	msg_kalman.data = m(1);
	pub_m_v.publish(msg_kalman);
	msg_kalman.data = x(0);
	pub_k_x.publish(msg_kalman);
	msg_kalman.data = x(1);
	pub_k_v.publish(msg_kalman);
}

void modelCB(const gazebo_msgs::ModelStates &msg) {
	gt(0) = msg.pose[2].position.x;
	gt(1) = msg.twist[2].linear.x;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "kalman");
	ros::Time::init();
	ros::Rate rr(10);
	ros::NodeHandle n;
	m << 0,
		 0;
	x << 0,
	     0;
	p << 1000,  0,
	     0,     1000;
	a << 1,     0.04,
	     0,     1;
	h << 1,     0,
	     0,     1;
	q << 1,     0,
	     0,     1;
	r << 0.001, 0.01,
	     0.01,  1.0;
	sub_model   = n.subscribe("/ackermann_vehicle_13/gazebo/model_states", 1, modelCB);
	sub_laser   = n.subscribe("/ackerman_vehicle/laser/scan", 1, laserCB);
	pub_m_x     = n.advertise<std_msgs::Float64>("/measurement_x", 10);
	pub_m_v     = n.advertise<std_msgs::Float64>("/measurement_v", 10);
	pub_k_x     = n.advertise<std_msgs::Float64>("/kalman_prediction_x", 10);
	pub_k_v     = n.advertise<std_msgs::Float64>("/kalman_prediction_v", 10);
	ros::Duration(10).sleep();

	while(ros::ok()) {
		ros::spinOnce();

		if(errors.size() == 250)
			printCovarianceMatrix();

		if(errors.size() < 251) {
			Eigen::Vector2f e = gt - m;
			errors.push_back(e);
		}

		rr.sleep();
	}

	return 0;
}
