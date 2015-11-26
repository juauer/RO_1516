#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

const double l1 = 1.0;
const double l2 = 0.9;
ros::Publisher pub_joints;
ros::Publisher pub_ee_y;
ros::Publisher pub_ee_z;
ros::Publisher pub_marker;
sensor_msgs::JointState msg_joints;
std_msgs::Float64 msg_ee;
visualization_msgs::Marker msg_marker;
double q1   = -M_PI * 1.0 / 8.0;
double q2   =  M_PI * 3.0 / 4.0;
double ee_y = -0.5;
double ee_z =  0.5;

void inverseTF(double dy, double dz, double *dq1, double *dq2) {
	const double j11 = -ee_z;
	const double j12 = -l2 * cos(q1 + q2);
	const double j21 = ee_y;
	const double j22 = -l2 * sin(q1 + q2);
	const double abs = j11*j22 - j12*j21;

	// TODO deal with singularities if abs near 0

	*dq1 = (j22*dy - j12*dz) / abs;
	*dq2 = (j11*dz - j21*dy) / abs;
}

void publishAll() {
	msg_joints.header.stamp = ros::Time::now();
	msg_joints.position[0]  = q1;
	msg_joints.position[1]  = q2;
	pub_joints.publish(msg_joints);
	msg_ee.data = ee_y;
	pub_ee_y.publish(msg_ee);
	msg_ee.data = ee_z;
	pub_ee_z.publish(msg_ee);
	geometry_msgs::Point p;
	p.x                     = 0;
	p.y                     = ee_y;
	p.z                     = ee_z;
	msg_marker.header.stamp = ros::Time::now();
	msg_marker.points.push_back(p);
	pub_marker.publish(msg_marker);
}

int main(int argc, char** argv) {
	int its     = atoi(argv[1]);
	double rate = std::min(32.0, std::max(1.0, its / 8.0));
	ros::init(argc, argv, "trajectory");
	ros::NodeHandle n;
	ros::Rate r(rate);
	tf::TransformListener listener_tf;
	tf::StampedTransform transform;
	pub_joints = n.advertise<sensor_msgs::JointState>(n.resolveName("/joint_states_forced"), 1);
	pub_ee_y   = n.advertise<std_msgs::Float64>(n.resolveName("/ee_y"), 1);
	pub_ee_z   = n.advertise<std_msgs::Float64>(n.resolveName("/ee_z"), 1);
	pub_marker = n.advertise<visualization_msgs::Marker>(n.resolveName("/marker"), 1);
	msg_joints.name.resize(2);
	msg_joints.position.resize(2);
	msg_joints.name[0]            = "joint_base_first";
	msg_joints.name[1]            = "joint_first_second";
	msg_marker.header.frame_id    = "/base_link";
	msg_marker.type               = visualization_msgs::Marker::LINE_STRIP;
	msg_marker.action             = visualization_msgs::Marker::ADD;
	msg_marker.pose.orientation.w = 1.0;
	msg_marker.scale.x            = 0.05;
	msg_marker.scale.y            = 0.05;
	msg_marker.scale.z            = 0.05;
	msg_marker.color.a            = 1.0;
	msg_marker.color.r            = 0.0;
	msg_marker.color.g            = 1.0;
	msg_marker.color.b            = 0.0;
	publishAll();

	double corners[4][2] = {{-0.5,-0.5}, {0.5,-0.5}, {0.5,0.5}, {-0.5,0.5}};
	
	for(int c=0; ; c=(c+1)%4) {
		int i        = 0;
		double tar_y = corners[c][0];
		double tar_z = corners[c][1];

		while(ros::ok() && i<its) {
			try {
				listener_tf.lookupTransform("/base_link", "/virtual_link", ros::Time(0), transform);
				ee_y      = transform.getOrigin().y();
				ee_z      = transform.getOrigin().z();
				double ip = i / (double)its;
				double dq1, dq2;
				inverseTF(ip * (tar_y-ee_y), ip * (tar_z-ee_z), &dq1, &dq2);
				q1 += dq1;
				q2 += dq2;
				publishAll();
				++i;
				r.sleep();
			}
			catch(tf::TransformException e){}
		}
	}

	return 0;
}
