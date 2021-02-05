/**
MIT License (modified)

Copyright (c) 2018 The Trustees of the University of Pennsylvania
Authors:
Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <map>
#include <string>
#include "odom_publisher.h"


// Model callback
void modelCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
	int des_index = 0;

	// Determine the index of the robot from all the models included
	if (robot_name == "minitaur") {
		for (int j=0 ; j<msg->name.size() ; j++) {
			if (msg->name[j] == "minitaur_constrained::base_chassis_link_dummy") {
				des_index = j;
			}
		}
	} else if (robot_name == "vision60") {
		for (int j=0 ; j<msg->name.size() ; j++) {
			if (msg->name[j] == "vision60::body") {
				des_index = j;
			}
		}
	}

    // Find orientation and position of robot in the global frame
	tf::Quaternion robot_to_global_orientation_quaternion = tf::Quaternion(msg->pose[des_index].orientation.x,
						  				  	                               msg->pose[des_index].orientation.y,
						  				  	                               msg->pose[des_index].orientation.z,
						  				  	                               msg->pose[des_index].orientation.w);
    tf::Matrix3x3 robot_to_global_orientation = tf::Matrix3x3(robot_to_global_orientation_quaternion);
    tf::Vector3 robot_to_global_position = tf::Vector3(msg->pose[des_index].position.x, msg->pose[des_index].position.y, msg->pose[des_index].position.z);

    // Find robot pose in the global frame
    tf::Pose robot_to_global = tf::Pose(robot_to_global_orientation, robot_to_global_position);

    // Find orientation and position of child in the robot frame
    tf::Matrix3x3 child_to_robot_orientation;
    child_to_robot_orientation.setRPY(child_frame_roll, child_frame_pitch, child_frame_yaw);
    tf::Vector3 child_to_robot_position = tf::Vector3(child_frame_x, child_frame_y, child_frame_z);

    // Find child pose in the robot frame
    tf::Pose child_to_robot = tf::Pose(child_to_robot_orientation, child_to_robot_position);

    // Store the first pose of the odom frame in the global Gazebo frame - Runs only once
    if (flag_first) {
        odom_to_global = robot_to_global * child_to_robot;
        odom_to_global_pos = odom_to_global.getOrigin();
        odom_to_global_rot = odom_to_global.getRotation();
        flag_first = false;
    }

    // Find pose of child frame in odom frame
    tf::Pose child_to_odom = odom_to_global.inverse()*robot_to_global*child_to_robot;

    // Initialize Odometry message
    nav_msgs::Odometry msg_odometry;

    // Populate odometry message header
    msg_odometry.header.stamp = ros::Time::now();
    msg_odometry.header.frame_id = odom_frame;

    // Populate odometry message
    msg_odometry.child_frame_id = child_frame;
    msg_odometry.pose.pose.position.x = child_to_odom.getOrigin().getX();
    msg_odometry.pose.pose.position.y = child_to_odom.getOrigin().getY();
    msg_odometry.pose.pose.position.z = child_to_odom.getOrigin().getZ();
    msg_odometry.pose.pose.orientation.x = child_to_odom.getRotation().getX();
    msg_odometry.pose.pose.orientation.y = child_to_odom.getRotation().getY();
    msg_odometry.pose.pose.orientation.z = child_to_odom.getRotation().getZ();
    msg_odometry.pose.pose.orientation.w = child_to_odom.getRotation().getW();

	// Get angular twist
	tf::Vector3 vector_angular(msg->twist[des_index].angular.x, msg->twist[des_index].angular.y, msg->twist[des_index].angular.z);
	tf::Vector3 rotated_vector_angular = tf::quatRotate(((robot_to_global*child_to_robot).inverse()).getRotation(), vector_angular);
	msg_odometry.twist.twist.angular.x = rotated_vector_angular.getX();
	msg_odometry.twist.twist.angular.y = rotated_vector_angular.getY();
	msg_odometry.twist.twist.angular.z = rotated_vector_angular.getZ();

	// Get linear twist
	tf::Vector3 vector_linear(msg->twist[des_index].linear.x, msg->twist[des_index].linear.y, msg->twist[des_index].linear.z);
	tf::Vector3 rotated_vector_linear = tf::quatRotate(((robot_to_global*child_to_robot).inverse()).getRotation(), vector_linear);
	msg_odometry.twist.twist.linear.x = rotated_vector_linear.getX();
	msg_odometry.twist.twist.linear.y = rotated_vector_linear.getY();
	msg_odometry.twist.twist.linear.z = rotated_vector_linear.getZ();

    // Publish odometry
    odomPub.publish(msg_odometry);

    // Publish tf transform
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(child_to_odom.getOrigin().getX(), child_to_odom.getOrigin().getY(), child_to_odom.getOrigin().getZ()));
    transform.setRotation(child_to_odom.getRotation());
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_frame, child_frame));

	return;
}

int main(int argc, char **argv) {
    // Initialize ROS if necessary
  	if (!ros::isInitialized()) {
    	ros::init(argc, argv, "odom_publisher", ros::init_options::NoSigintHandler);
  	}

	// Initialize node
  	rosNode.reset(new ros::NodeHandle("odom_publisher"));
    
    // Get parameters
    ros::param::get("~robot_name", robot_name);
    ros::param::get("~child_frame", child_frame);
    ros::param::get("~odom_frame", odom_frame);
    ros::param::get("~child_frame_x", child_frame_x);
    ros::param::get("~child_frame_y", child_frame_y);
    ros::param::get("~child_frame_z", child_frame_z);
    ros::param::get("~child_frame_roll", child_frame_roll);
    ros::param::get("~child_frame_pitch", child_frame_pitch);
    ros::param::get("~child_frame_yaw", child_frame_yaw);

    // Initialize subscriber
    modelSub = rosNode->subscribe("/gazebo/link_states", 1, &modelCallback);

    // Initialize publisher
    odomPub = rosNode->advertise<nav_msgs::Odometry>("/robot_odom", 1);

	while (ros::ok()) {
		// Set ROS rate
		ros::Rate r(60);

		// Spin once
		ros::spinOnce();

		r.sleep();
	}

	return 0;
}
