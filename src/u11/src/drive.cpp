#include <ros/ros.h>
#include <tf/tf.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include "PID.h"
#include "rrt.h"

ros::Subscriber sub_odo;
ros::Publisher pub_cx;
ros::Publisher pub_cy;
ros::Publisher pub_tx;
ros::Publisher pub_ty;
ros::Publisher pub_drive;
std::vector<RRT::Pose> path;
int i        = 0;
float rate   = 10.0;
double speed = 1.0;
double cx    = 0.0;
double cy    = 0.0;
double tx    = 0.0;
double ty    = 0.0;
double error = 0.0;

void odoCB(const nav_msgs::Odometry &msg) {
	cx = msg.pose.pose.position.x;
	cy = msg.pose.pose.position.y;
	double uyaw = tf::getYaw(msg.pose.pose.orientation);
	double syaw = uyaw <= M_PI ? uyaw : uyaw - 2.0 * M_PI;
	double dist = sqrtf( (tx-cx)*(tx-cx) + (ty-cy)*(ty-cy) );

	if(i == 0 && dist < 2) {
		speed = 0;
		error = 0;
		return;
	}

	if(dist < 2) {
		--i;
		tx = path[i].x;
		ty = path[i].y;
	}

	error = (dist==0.0) ? 0 : atan2(ty - cy, tx - cx) - syaw;
}

bool blocked(RRT::Pose &p) {
	if(p.x < 75 || p.x > 100 || p.y < -25 || p.y > 100)
		return false;

	return true;
}

RRT::Pose sample() {
	return RRT::Pose(rand()%170 - 10, rand()%145 - 35);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "u11_drive");
	ros::Time::init();
	ros::Rate r(rate);
	ros::NodeHandle n;
	RRT::Pose start(0, 0);
	RRT::Pose goal(150, 75);
	PID pid(1, 0, 1, 0);
	path       = RRT::rrt(start, goal, 1000, 5.0, &sample, &blocked);
	i          = path.size() - 1;
	tx         = path[i].x;
	ty         = path[i].y;
	sub_odo    = n.subscribe("/ackermann_vehicle/odom", 10, odoCB);
	pub_cx     = n.advertise<std_msgs::Float64>("/u11/cur_x", 10);
	pub_cy     = n.advertise<std_msgs::Float64>("/u11/cur_y", 10);
	pub_tx     = n.advertise<std_msgs::Float64>("/u11/tar_x", 10);
	pub_ty     = n.advertise<std_msgs::Float64>("/u11/tar_y", 10);
	pub_drive  = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 10);
	std_msgs::Float64 msg_plot;
	ackermann_msgs::AckermannDriveStamped msg_drive;
	msg_drive.drive.jerk                    = 0;
	msg_drive.drive.steering_angle_velocity = 1;
	msg_drive.drive.acceleration            = 2;
	ros::Duration(5).sleep();

	while(ros::ok()) {
		double pidout = pid.control(error);
		msg_plot.data = cx;
		pub_cx.publish(msg_plot);
		msg_plot.data = cy;
		pub_cy.publish(msg_plot);
		msg_plot.data = tx;
		pub_tx.publish(msg_plot);
		msg_plot.data = ty;
		pub_ty.publish(msg_plot);
		msg_drive.header.stamp         = ros::Time(0);
		msg_drive.drive.steering_angle = std::max(-M_PI/2, std::min(pidout, M_PI/2));
		msg_drive.drive.speed          = speed;
		pub_drive.publish(msg_drive);
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
