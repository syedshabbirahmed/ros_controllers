#include <iostream>
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
#include "inverse_kinematic_controller/move_robot.h"
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/duration.h>
#include <ros/ros.h>

class Server
{
  // init the node
  public:

	// target pose
    long double xf, yf, zf, T;

	// current pose
	long double xs, ys, zs;

	pinocchio::Model model;
	int dim_joints; const int JOINT_ID = 7;
	pinocchio::Data data;

	// joint state and end effector position/velocity
	Eigen::VectorXd joint_pos, joint_vel;
	Eigen::MatrixXd jacobian_local_world, jacobian_pseudo_inv;
	Eigen::MatrixXd N;

	Eigen::VectorXd x_dot;
	Eigen::VectorXd q, q_dot, q0, q0_dot;
	Eigen::MatrixXd dt;

	bool flag = false;
	bool pub_flag = false;

	// pose messages
	sensor_msgs::JointState msg;
	std_msgs::Float64MultiArray pub_msg;

	// to record the time
	ros::Time ros_start_time;
	ros::Duration total_service_time;
	ros::Time ros_end_time;

	// create node handle to publish pose
	ros::NodeHandle node;
	ros::Publisher pose_pub;

	// functions
	bool service_callback( inverse_kinematic_controller::move_robot::Request  &req,
						   inverse_kinematic_controller::move_robot::Response &res);
	void subscriber_callback(sensor_msgs::JointState msg);
	void initial_pose();
	void update_pose();

	Server() {

		// change this to your directory
		std::string urdf_file_name = "/home/syedshabbir/catkin_ws/src/ros_kortex/kortex_description/urdf/gen3.urdf";
		pinocchio::urdf::buildModel(urdf_file_name, model, false);				// read the URDF file
		data = pinocchio::Data(model);	
		dim_joints = model.nq ;

		// initialize the joint positions
		joint_pos = Eigen::VectorXd::Zero(dim_joints); // pinocchio::neutral(model) ; // neutral is the default configuration
		joint_vel = Eigen::VectorXd::Zero(dim_joints);

		// *************** Jacobian in the world frame *******************************************
		jacobian_local_world = Eigen::MatrixXd::Zero(6,dim_joints) ;
		jacobian_pseudo_inv = Eigen::MatrixXd::Zero(dim_joints,6) ;

		// *************** Initialize end effector velocity and joint states ***************************************************
		x_dot = Eigen::VectorXd::Zero(3);
		q_dot = Eigen::VectorXd::Zero(dim_joints);

		// *************** Choose the dt ***************************************************
		dt = Eigen::MatrixXd::Identity(dim_joints, dim_joints);
		dt = dt * 0.01;

		// publish to topic "/gen3/joint_group_position_controller/command"
  		pose_pub = node.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_position_controller/command", 3);
	}
};


// service callback
bool Server::service_callback( inverse_kinematic_controller::move_robot::Request  &req,
						   inverse_kinematic_controller::move_robot::Response &res) {
	
	// get target pose
	xf = (long double) req.x;
	yf = (long double) req.y;
	zf = (long double) req.z;
	T  = (long double) req.T;
	
	// set flag to true once service is called
	res.flag = flag = true;
	return true;
}

// subscriber callback
void Server::subscriber_callback(sensor_msgs::JointState msg) {
	
	// save global model state
	joint_pos(0) = msg.position[0];
	joint_pos(1) = msg.position[1];
	joint_pos(2) = msg.position[2];
	joint_pos(3) = msg.position[3];
	joint_pos(4) = msg.position[4];
	joint_pos(5) = msg.position[5];
	joint_pos(6) = msg.position[6];
	joint_vel(0) = msg.velocity[0];
	joint_vel(1) = msg.velocity[1];
	joint_vel(2) = msg.velocity[2];
	joint_vel(3) = msg.velocity[3];
	joint_vel(4) = msg.velocity[4];
	joint_vel(5) = msg.velocity[5];
	joint_vel(6) = msg.velocity[6];
}

void Server::initial_pose(){

	// FORWARD KINEMATICS
	pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);							// forward kinematics
	pinocchio::SE3 pose_now = data.oMi[JOINT_ID]  ;												// end-effector pose
	xs = pose_now.translation()[0];
	ys = pose_now.translation()[1];
	zs = pose_now.translation()[2];
	q0 = joint_pos;
	q = joint_pos;

	// get the start time, end time and total service time
	ros_start_time = ros::Time::now();
	total_service_time = ros::Duration(T);
	ros_end_time = ros_start_time + total_service_time;
	
	flag = false;
	pub_flag = true;
}

void Server::update_pose(){

	if (ros::Time::now() < ros_end_time) {

		// update the time reemaining
		ros::Duration duration = ros::Time::now() - ros_start_time;

		// compute timescale for third order polynomial
		long double s = 6/(total_service_time.toSec()*total_service_time.toSec()) * duration.toSec() \
					- 6/(total_service_time.toSec()*total_service_time.toSec()*total_service_time.toSec()) \
					* duration.toSec() * duration.toSec();

		// compute the updated end-effector velocity
		x_dot(0) = s * (xf - xs);
		x_dot(1) = s * (yf - ys);
		x_dot(2) = s * (zf - zs);

		// *************** Jacobian in the world frame *******************************************
		pinocchio::computeAllTerms(model, data, joint_pos, joint_vel);
		pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;

		// *************** Pseudo Inverse of Jacobian *******************************************
		jacobian_pseudo_inv = jacobian_local_world.block(0,0,3,7).completeOrthogonalDecomposition().pseudoInverse() ;

		// Redundancy resolution
		N = Eigen::MatrixXd::Identity(dim_joints, dim_joints) - (jacobian_pseudo_inv * jacobian_local_world.block(0,0,3,7)) ;
		q0_dot = q0 - q;

		// *************** Joint Velocity ********************************************************
		q_dot = jacobian_pseudo_inv*x_dot + N * q0_dot;

		// *************** Joint Position ********************************************************
		q = q + dt * q_dot;

		// publish the joint positions
		pub_msg.data.clear();
		pub_msg.data.push_back(q(0));
		pub_msg.data.push_back(q(1));
		pub_msg.data.push_back(q(2));
		pub_msg.data.push_back(q(3));
		pub_msg.data.push_back(q(4));
		pub_msg.data.push_back(q(5));
		pub_msg.data.push_back(q(6));
		pose_pub.publish(pub_msg);
		q_dot = Eigen::VectorXd::Zero(dim_joints);
	}
}

int main(int argc, char **argv) {

	// initialize node
	ros::init(argc, argv, "inverse_kinematic_controller_node");

	// create node handle
	ros::NodeHandle node;

	// specify the frequency to 100HZ
	ros::Rate loopRate(100) ;

	// advertise the service
	Server server;

	// advertise a service "/cubic_polynomial_planner/move_robot"
	ros::ServiceServer service = node.advertiseService("/cubic_polynomial_planner/move_robot", &Server::service_callback, &server);
	
	// subscribe to topic "/gazebo/model_states"
	ros::Subscriber pose_sub = node.subscribe("/gen3/joint_states", 100, &Server::subscriber_callback, &server);

	// Serive move_robot is created
	ROS_INFO("Server: a service to move the robot is created");
	
	// ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while(ros::ok()){
		
		// if service is called
		if (server.flag) {
			// find the initial robot pose
			server.initial_pose();
		}

		// if service is called
		if (server.pub_flag) {

			// update the pose
			server.update_pose();

		}

		ros::spinOnce(); // will call all the callbacks waiting to be called at that point in time

		loopRate.sleep();
	}

	return 0 ;
}