#include "ros/ros.h"
#include "ros/console.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "astar.h"

Pose start(0, 0, 1, 0);
Pose  goal(0, 0, 1, 0);
float v, w;

// liegt p in einem hindernis?
inline bool insideObstacle(Pose &p) {
	if(p.x < 15 || p.x > 20 || p.y < -5 || p.y > 20)
		return false;

	return true;
}

// alle von p mit schrittweite v erreichbaren posen
std::vector<Pose> neighbors(Pose &p) {
	std::vector<Pose> ns;
	Pose l(p.x+sin(p.ox-w), p.y+cos(p.oy+w), p.ox-w, p.oy+w);
	Pose r(p.x+sin(p.ox+w), p.y+cos(p.oy-w), p.ox+w, p.oy-w);
	Pose c(p.x+sin(p.ox), p.y+cos(p.oy), p.ox, p.oy);

	if(!insideObstacle(l))
		ns.push_back(l);

	if(!insideObstacle(r))
		ns.push_back(r);

	if(!insideObstacle(c))
		ns.push_back(c);

	return ns;
}

// tatsaechliche kosten von p1 nach p2
double g(Pose &p1, Pose &p2) {
	return v;
}

// geschaetzte kosten von p nach goal
double h(Pose &p) {
	double dx = goal.x - p.x;
	double dy = goal.y - p.y;
	return sqrt(dx*dx + dy*dy);
}

// ist p = goal?
bool goalReached(Pose &p) {
	if(abs(goal.x - p.x) < 1 && abs(goal.y - p.y) < 1)
		return true;

	return false;
}

int main(int argc, char **argv) {
	goal = Pose(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]));
	v    = M_PI * atof(argv[5]);
	w    = v / 4;
	ros::init(argc, argv, "show_path");
	ros::NodeHandle n;
	ros::Publisher pub_path = n.advertise<nav_msgs::Path>("/path", 2048);
	ros::Publisher pub_obst = n.advertise<visualization_msgs::Marker>("/obstacle", 1);
	visualization_msgs::Marker msg_obst;
	msg_obst.header.frame_id    = "/map";
	msg_obst.type               = visualization_msgs::Marker::CUBE;
	msg_obst.action             = visualization_msgs::Marker::ADD;
	msg_obst.pose.position.x    = 17.5;
	msg_obst.pose.position.y    = 7.5;
	msg_obst.pose.orientation.w = 1.0;
	msg_obst.scale.x            = 2.5;
	msg_obst.scale.y            = 12.5;
	msg_obst.scale.z            = 1.0;
	msg_obst.color.a            = 1.0;
	msg_obst.color.r            = 0.0;
	msg_obst.color.g            = 0.0;
	msg_obst.color.b            = 1.0;
	std::vector<geometry_msgs::PoseStamped> plan;
	AStar astar(start, goal, &neighbors, &g, &h, &goalReached);
	Node *node = astar.open.top();

	while(node != NULL) {
		geometry_msgs::PoseStamped step;
		tf::Quaternion quat     = tf::createQuaternionFromYaw(atan2(node->pose.oy, node->pose.ox));
		step.pose.position.x    = node->pose.x;
		step.pose.position.y    = node->pose.y;
		step.pose.orientation.x = quat.x();
		step.pose.orientation.y = quat.y();
		step.pose.orientation.z = quat.z();
		step.pose.orientation.w = quat.w();
		plan.push_back(step);
		node = node->parent;
	}

	ROS_INFO("Nodes in OPEN: %lu", astar.open.size());
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
