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
#include "inverse_dynamics_controller/move_robot.h"
#include <ros/duration.h>
#include <ros/ros.h>

class Server
{
  // init the node
  private:

	// target pose
    double Katrr, Dtask, Ktask, Djoint, Kjoint;

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

	// pose messages
	sensor_msgs::JointState msg;
	std_msgs::Float64MultiArray pub_msg;

	// create node handle to publish pose
	ros::NodeHandle node;
	ros::Publisher pose_pub;

  public:

	bool flag = false;
	bool pub_flag = false;
    
	// functions
    Server();
	bool service_callback( inverse_dynamics_controller::move_robot::Request  &req,
						   inverse_dynamics_controller::move_robot::Response &res);
	void subscriber_callback(sensor_msgs::JointState msg);
	void initial_pose();
	void dynamics_controller();
};