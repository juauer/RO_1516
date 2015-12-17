#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
//#include <tf/quaternion.h>
#include <tf/tf.h>
#include "geometry_msgs/Quaternion.h"

//generic C/C++ include


int main(int argc, char **argv)
{
    ros::init(argc,argv,"showig_path");
    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<std_msgs::Float64 >("/drc_vehicle_xp900/gas_pedal/cmd",1000);

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 1000);
    std::vector<geometry_msgs::PoseStamped> plan;
    nav_msgs::Path gui_path;

    //ros::Rate loop_rate(1);
   for (int i=0; i<200; i++){
     geometry_msgs::PoseStamped new_goal;
     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54*i);
      //tf::Quaternion goal_quat = tf::createQuaternionFromRPY(0,0,1);
      

      new_goal.pose.position.x = -2.5+(0.05*i);
      new_goal.pose.position.y = -3.5+(0.05*i);

      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();

      // new_goal.pose.orientation.x = 0;
      // new_goal.pose.orientation.y =0;
      // new_goal.pose.orientation.z = 0;
      // new_goal.pose.orientation.w =1;

      plan.push_back(new_goal);
   }
   gui_path.poses.resize(plan.size());

    if(!plan.empty()){
          gui_path.header.frame_id = "map";
          gui_path.header.stamp = plan[0].header.stamp;
    }

    for(unsigned int i=0; i < plan.size(); i++){
          gui_path.poses[i] = plan[i];
    }
 

    while (ros::ok())
    {   
        path_pub.publish(gui_path);
        ros::spinOnce();
    }
    return 0;
}
