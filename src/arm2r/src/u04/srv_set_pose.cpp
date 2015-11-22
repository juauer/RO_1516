#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "arm2r/SetPose.h"

ros::Publisher pub_joints;
sensor_msgs::JointState msg_joints;

bool setPose(arm2r::SetPose::Request &req, arm2r::SetPose::Response &res) {
	msg_joints.header.stamp = ros::Time::now();
	msg_joints.position[0]  = 0.5 * M_PI * (req.pose - 2);
	msg_joints.position[1]  = 0;
	pub_joints.publish(msg_joints);
	res.success = 0;
	return true;
}
    
int main(int argc, char** argv) {
	ros::init(argc, argv, "srv_set_pose");
	ros::NodeHandle n;
	ros::ServiceServer s = n.advertiseService("set_pose", setPose);
	pub_joints = n.advertise<sensor_msgs::JointState>(n.resolveName("/joint_states_arm"), 1);
	msg_joints.name.resize(2);
	msg_joints.position.resize(2);
	msg_joints.name[0] ="joint_base_first";
	msg_joints.name[1] ="joint_first_second";
	msg_joints.header.stamp = ros::Time::now();
	msg_joints.position[0]  = 0;
	msg_joints.position[1]  = 0;
	pub_joints.publish(msg_joints);
	ros::spin();
	return 0;
}
