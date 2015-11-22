#include <string>
#include <ros/ros.h>
#include "arm2r/SetPose.h"

/*
** usage: rosrun arm2r arm2r_client 1|2|3
**   1: point right
**   2: point upwards
**   3: point left
*/
int main(int argc, char** argv) {
	ros::init(argc, argv, "client");
	ros::NodeHandle n;
	ros::ServiceClient c = n.serviceClient<arm2r::SetPose>("set_pose");
	arm2r::SetPose r;
	r.request.pose = atoi(argv[1]);
	c.call(r);
	return 0;
}
