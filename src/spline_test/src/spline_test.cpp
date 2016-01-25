#include "ros/ros.h"
#include <string>
#include <gtest/gtest.h>
#include <ecl/containers.hpp>
#include <ecl/geometry.hpp>
#include "nav_msgs/Path.h"

using std::string;
using ecl::Array;
using ecl::CubicPolynomial;
using ecl::CubicSpline;

nav_msgs::Path subsample(CubicSpline cubic,float start_x, float end_x) {
	std::vector<geometry_msgs::PoseStamped> plan;
	nav_msgs::Path gui_path;
	geometry_msgs::PoseStamped new_goal;
	int sampling_num = 200;

	for (int i=0; i<sampling_num; i++){
		new_goal.pose.position.x    = i * (end_x-start_x) / sampling_num + start_x;
		new_goal.pose.position.y    = cubic(i * (end_x-start_x) / sampling_num + start_x);
		new_goal.pose.orientation.x = 0;
		new_goal.pose.orientation.y = 0;
		new_goal.pose.orientation.z = 0;
		new_goal.pose.orientation.w = 1;
		new_goal.header.stamp       = ros::Time::now();
		new_goal.header.seq++;
		plan.push_back(new_goal);
	}

	gui_path.poses.resize(plan.size());

	if(!plan.empty()) {
		gui_path.header.frame_id = "map";
		gui_path.header.stamp    = plan[0].header.stamp;
	}

	for(unsigned int i=0; i < plan.size(); i++)
		gui_path.poses[i] = plan[i];

	return gui_path;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_spline");
	Array<double> x_set(10);
	Array<double> y_set(10);
//	x_set <<  0.688792, 1.15454, 1.67894, 2.1,  2.7, 3.1, 3.6, 4,   5,   6;
//	y_set << -0.75,    -1.2,    -0.50,   -1.4, -1,   0,   0.1, 1.3, 0.3, 1.0;

	for(int i=0; i<10; ++i) {
		double x = 6.0f * i / 9.0f;
		double y = (rand()%400) / 100.0f - 2.0f;
		x_set[i] = x;
		y_set[i] = y;
	}

	double ydot_0      = 1; // -0.5 / x_set[0];
	double ydot_f      = 1; // 1.04 / 3.72;
	CubicSpline cubic1 = CubicSpline::Natural(x_set, y_set);
	CubicSpline cubic2 = CubicSpline::DerivativeHeuristic(x_set, y_set, ydot_0, ydot_f);
	CubicSpline cubic3 = CubicSpline::ContinuousDerivatives(x_set, y_set, ydot_0, ydot_f);

//	const CubicPolynomial &p1 = cubic1.polynomials()[0];
//	const CubicPolynomial &p2 = cubic1.polynomials()[1];
//	const CubicPolynomial &p3 = cubic1.polynomials()[2];
//	std::cout << cubic1 << std::endl;
//	std::cout << "Value         : " << cubic1(0.8) << std::endl;
//	std::cout << "Coffiecinets         : " << p1.coefficients()<< std::endl;

	ros::NodeHandle n;
	ros::Publisher path_pub1 = n.advertise<nav_msgs::Path>("/path_Natural", 1000);
	ros::Publisher path_pub2 = n.advertise<nav_msgs::Path>("/path2_DerivativeHeuristic", 1000);
	ros::Publisher path_pub3 = n.advertise<nav_msgs::Path>("/path3_ContinuousDerivatives", 1000);
	nav_msgs::Path gui_path1;
	nav_msgs::Path gui_path2;
	nav_msgs::Path gui_path3;

 	while(ros::ok()) {
 		gui_path1 = subsample(cubic1, 0/*.688792*/, 6);
 		path_pub1.publish(gui_path1);
 		gui_path2 = subsample(cubic2, 0/*.688792*/, 6);
 		path_pub2.publish(gui_path2);
 		gui_path3 = subsample(cubic3, 0/*.688792*/, 6);
 		path_pub3.publish(gui_path3);
 		ros::spinOnce();
	}

 	return 0;
}

