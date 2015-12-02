ackermann_vehicle
=================

ROS packages for simulating a vehicle with Ackermann steering

INSTALL
__________________
first I tried to add the package to my repo simply and compile it. 
But it does not work!
So I tried to install it using ros-indigo-ackermann-vehicle 
again it does not install some files like urdf and ...
I tried to use ros-indigo-ackermann-vehicle* which fixed my errors.


Move the robot in terminal
________________________
after running the: roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch
you can try to publisg the "ackermann_vehicle/ackermann_cmd" to move the robot in gazebo:

rostopic pub -r 10 ackermann_vehicle/ackermann_cmd ackermann_msgs/AckermannDriveStamped '{header: auto, drive: [1,0,1,0,0]}

My log
__________________________
zargol@zargol:~$ roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch 
... logging to /home/zargol/.ros/log/10499d00-8ed9-11e5-936a-ac72898d8afb/roslaunch-zargol-7121.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://zargol:54872/

SUMMARY
========

PARAMETERS
 * /ackermann_vehicle/ackermann_controller/cmd_timeout: 0.5
 * /ackermann_vehicle/ackermann_controller/left_front_wheel/axle_controller_name: left_front_axle_c...
 * /ackermann_vehicle/ackermann_controller/left_front_wheel/diameter: 0.14605
 * /ackermann_vehicle/ackermann_controller/left_front_wheel/steering_controller_name: left_steering_ctrlr
 * /ackermann_vehicle/ackermann_controller/left_front_wheel/steering_link_name: left_front_wheel
 * /ackermann_vehicle/ackermann_controller/left_rear_wheel/axle_controller_name: left_rear_axle_ctrlr
 * /ackermann_vehicle/ackermann_controller/left_rear_wheel/diameter: 0.14605
 * /ackermann_vehicle/ackermann_controller/left_rear_wheel/link_name: left_rear_wheel
 * /ackermann_vehicle/ackermann_controller/right_front_wheel/axle_controller_name: right_front_axle_...
 * /ackermann_vehicle/ackermann_controller/right_front_wheel/diameter: 0.14605
 * /ackermann_vehicle/ackermann_controller/right_front_wheel/steering_controller_name: right_steering_ctrlr
 * /ackermann_vehicle/ackermann_controller/right_front_wheel/steering_link_name: right_front_wheel
 * /ackermann_vehicle/ackermann_controller/right_rear_wheel/axle_controller_name: right_rear_axle_c...
 * /ackermann_vehicle/ackermann_controller/right_rear_wheel/diameter: 0.14605
 * /ackermann_vehicle/ackermann_controller/right_rear_wheel/link_name: right_rear_wheel
 * /ackermann_vehicle/ackermann_controller/shock_absorbers: [{'controller_nam...
 * /ackermann_vehicle/robot_description: <?xml version="1....
 * /ackermann_vehicle/vehicle_state_publisher/publish_frequency: 30.0
 * /rosdistro: indigo
 * /rosversion: 1.11.10
 * /use_sim_time: True

NODES
  /ackermann_vehicle/
    ackermann_controller (ackermann_vehicle_gazebo/ackermann_controller)
    controller_spawner (controller_manager/spawner)
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
    spawn_vehicle (gazebo_ros/spawn_model)
    vehicle_state_publisher (robot_state_publisher/robot_state_publisher)

auto-starting new master
process[master]: started with pid [7136]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 10499d00-8ed9-11e5-936a-ac72898d8afb
process[rosout-1]: started with pid [7155]
started core service [/rosout]
process[ackermann_vehicle/vehicle_state_publisher-2]: started with pid [7179]
process[ackermann_vehicle/gazebo-3]: started with pid [7180]
process[ackermann_vehicle/gazebo_gui-4]: started with pid [7184]
process[ackermann_vehicle/spawn_vehicle-5]: started with pid [7190]
process[ackermann_vehicle/controller_spawner-6]: started with pid [7226]
process[ackermann_vehicle/ackermann_controller-7]: started with pid [7229]
Gazebo multi-robot simulator, version 2.2.6
Copyright (C) 2012-2014 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[ INFO] [1447949867.138100771]: Finished loading Gazebo ROS API Plugin.
Msg Waiting for master[ INFO] [1447949867.138979999]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...

Msg Connected to gazebo master @ http://127.0.0.1:11345
Msg Publicized address: 10.37.76.2
Gazebo multi-robot simulator, version 2.2.6
Copyright (C) 2012-2014 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

Msg Waiting for master
Msg Connected to gazebo master @ http://127.0.0.1:11345
Msg Publicized address: 10.37.76.2
[ INFO] [1447949868.215598060, 0.088000000]: Loading gazebo_ros_control plugin
[ INFO] [1447949868.215756714, 0.088000000]: Starting gazebo_ros_control plugin in namespace: /ackermann_vehicle/
[ INFO] [1447949868.217878692, 0.088000000]: gazebo_ros_control plugin is waiting for model URDF in parameter [/ackermann_vehicle/robot_description] on the ROS param server.
[ackermann_vehicle/spawn_vehicle-5] process has finished cleanly
log file: /home/zargol/.ros/log/10499d00-8ed9-11e5-936a-ac72898d8afb/ackermann_vehicle-spawn_vehicle-5*.log
[ INFO] [1447949868.485262440, 0.088000000]: Loaded gazebo_ros_control.
Error [Param.cc:181] Unable to set value [1,0471975511965976] for key[horizontal_fov]
Error [Param.cc:181] Unable to set value [0,100000001] for key[near]

