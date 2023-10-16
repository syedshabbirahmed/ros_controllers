#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include </usr/include/eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64MultiArray.h>
#include <actionlib/server/simple_action_server.h>
#include <highlevel_controller/MoveToAction.h>
// #include "highlevel_controller/move_robot.h"
#include <ros/duration.h>
#include <ros/ros.h>

class Server
{
  // init the node
  private:

	// target pose
    double Katrr, Dtask_trans, Ktask_trans, Dtask_orien, Ktask_orien, Djoint, Kjoint;

	// current pose
	Eigen::VectorXd x_final;

	// urdf file
	std::string filename;

	pinocchio::Model model;
	int dim_joints; const int JOINT_ID = 7;
	pinocchio::Data data;

	// joint state and end effector position/velocity
	Eigen::VectorXd joint_pos, joint_vel;
	Eigen::MatrixXd jacobian_local_world, jacobian_dot, jacobian_pseudo_inv, jacobian_trans_pseudo_inv;
	Eigen::MatrixXd N;
	Eigen::MatrixXd M, h;

	Eigen::VectorXd x_dot;
	Eigen::MatrixXd dt;

	// potential field planner
	Eigen::VectorXd x_ref, x_ref_dot, x_ref_ddot;
	Eigen::VectorXd q_ref, q_ref_dot, q_ref_ddot;

	// task space controller
	Eigen::VectorXd x_fbk, x_fbk_dot, x_cmd_ddot, n, F_cmd, T_task;
	Eigen::MatrixXd A;

	// joint space controller
	Eigen::VectorXd q_tar;
	Eigen::VectorXd q_cmd_ddot, T_joint, T_null, T;
	Eigen::MatrixXd D, P, KatrrM;

	// orientation finder
	pinocchio::SE3 pose_ref;
	Eigen::VectorXd pose_err_local; // local position error in vector
	Eigen::VectorXd pose_err; // position error in vector

	// pose messages
	sensor_msgs::JointState msg;
	std_msgs::Float64MultiArray pub_msg;

	// create node handle to publish pose
	ros::NodeHandle& node;
	ros::Publisher pose_pub;
	ros::Subscriber pose_sub;

	ros::Rate loopRate;

	void move_to_callback(const highlevel_controller::MoveToGoalConstPtr& goal) ;
	actionlib::SimpleActionServer<highlevel_controller::MoveToAction> move_to_action_server;

  public:
	bool flag = true;
	bool pub_flag = false;
    
	// functions
    Server(ros::NodeHandle& node_handle);
	void feedbackCallback(sensor_msgs::JointState msg);
	void dynamics_controller();
};