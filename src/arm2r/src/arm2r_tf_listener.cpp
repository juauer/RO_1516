#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
    
int main(int argc, char** argv) {
	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::Publisher pub_joints = n.advertise<sensor_msgs::JointState>(n.resolveName("/calibrated/joint_states"), 10);
	ros::Publisher pub_link   = n.advertise<std_msgs::Float64>("link_virt", 10);
	sensor_msgs::JointState joint_state;
	std_msgs::Float64 link_d;
	ros::Rate loop_rate(10);
	tf::TransformListener listener;
	double th = 0;

	while (ros::ok()) {
		th = (th + 0.1) >= (2 * M_PI) ? 0 : (th + 0.1);
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(2);
		joint_state.position.resize(2);
		joint_state.name[0] ="joint_base_first";
		joint_state.position[0] = sin(th);
		joint_state.name[1] ="joint_first_second";
		joint_state.position[1] = cos(th);
		pub_joints.publish(joint_state);
		tf::StampedTransform transform;

		try {
			listener.lookupTransform("/virtual_link", "/base_link", ros::Time(0), transform);
			link_d.data = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().z(), 2));
			pub_link.publish(link_d);
		} catch (tf::TransformException ex) {}

		loop_rate.sleep();

	}

	return 0;
}
