#include "ros/ros.h"
#include "ros/console.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"

#include "rrt.h"

RRT::Pose start(0, 0);
RRT::Pose  goal(0, 0);

bool blocked(RRT::Pose &p) {
	if(p.x < 75 || p.x > 100 || p.y < -25 || p.y > 100)
		return false;

	return true;
}

RRT::Pose sample() {
	return RRT::Pose(rand()%170 - 10, rand()%145 - 35);
}

int main(int argc, char **argv) {
	goal = RRT::Pose(atof(argv[1]), atof(argv[2]));
	ros::init(argc, argv, "u11_draw");
	ros::NodeHandle n;
	ros::Publisher pub_path = n.advertise<nav_msgs::Path>("/path", 2048);
	ros::Publisher pub_obst = n.advertise<visualization_msgs::Marker>("/obstacle", 1);
	visualization_msgs::Marker msg_obst;
	msg_obst.header.frame_id    = "/map";
	msg_obst.type               = visualization_msgs::Marker::CUBE;
	msg_obst.action             = visualization_msgs::Marker::ADD;
	msg_obst.pose.position.x    = 87.5;
	msg_obst.pose.position.y    = 37.5;
	msg_obst.pose.orientation.w = 1.0;
	msg_obst.scale.x            = 25;
	msg_obst.scale.y            = 125;
	msg_obst.scale.z            = 1.0;
	msg_obst.color.a            = 1.0;
	msg_obst.color.r            = 0.0;
	msg_obst.color.g            = 0.0;
	msg_obst.color.b            = 1.0;
	std::vector<geometry_msgs::PoseStamped> plan;
	std::vector<RRT::Pose> rrt = RRT::rrt(start, goal, 1000, 5, &sample, &blocked);

	for(std::vector<RRT::Pose>::reverse_iterator node=rrt.rbegin(); node!=rrt.rend(); ++node) {
		ROS_INFO("PATH: %.2f, %.2f", node->x, node->y);
		geometry_msgs::PoseStamped step;
		tf::Quaternion quat     = tf::createQuaternionFromYaw(atan2(0, 1));
		step.pose.position.x    = node->x;
		step.pose.position.y    = node->y;
		step.pose.orientation.x = quat.x();
		step.pose.orientation.y = quat.y();
		step.pose.orientation.z = quat.z();
		step.pose.orientation.w = quat.w();
		plan.push_back(step);
	}

	nav_msgs::Path msg_path;
	msg_path.poses.resize(plan.size());
	msg_path.header.frame_id = "map";
	msg_path.header.stamp = plan[0].header.stamp;

	for(int i = 0; i < plan.size(); ++i)
		msg_path.poses[i] = plan[i];

	while (ros::ok()) {
        pub_path.publish(msg_path);
        pub_obst.publish(msg_obst);
        ros::spinOnce();
	}

	return 0;
}
