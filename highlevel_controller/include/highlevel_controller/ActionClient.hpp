#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <highlevel_controller/MoveToAction.h>
#include <control_msgs/GripperCommandAction.h>




class ActionClient {

	public:
		ActionClient(ros::NodeHandle& node_handle);
		void update();

	private:
		bool readParameters();
		bool success;

	 	// ROS node handle.
		ros::NodeHandle& node;

		//! ROS topic name to subscribe to.
		std::string feedback_topic_name;

		highlevel_controller::MoveToGoal action_goal;

		actionlib::SimpleActionClient<highlevel_controller::MoveToAction> move_to_action_client; //("go_to_home_configuration", true); // true -> don't need ros::spin()

		control_msgs::GripperCommandGoal squeeze_goal;

		actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client;

		int NUM_TARGETS 		  = 4 ;
		int NUM_GRIPPER_POSITIONS = 3 ;
		Eigen::MatrixXd target_translation;
		Eigen::MatrixXd gripper_position;

		enum GRASP_STATE {
		HOME ,
		DEFAULT_POSITION ,
		OPEN_GRIPPER ,
		PREGRASP_POSITION ,
		GRASP ,
		RELEASE_POSITION ,
		RELEASE ,
	};

};


