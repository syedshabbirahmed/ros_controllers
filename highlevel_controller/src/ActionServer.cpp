#include "highlevel_controller/ActionServer.hpp"

Server::Server(ros::NodeHandle& node_handle):node(node_handle),
									  move_to_action_server(node,"/move_to_action", boost::bind(&Server::move_to_callback, this, _1), false),
									  loopRate(1000)
									  {								
    node.getParam("/gen3/urdf_file_name",filename);
    node.getParam("gains/Katrr",         Katrr);
    node.getParam("gains/Dtask_trans",   Dtask_trans);
    node.getParam("gains/Ktask_trans",   Ktask_trans);
	node.getParam("gains/Dtask_orien",   Dtask_orien);
    node.getParam("gains/Ktask_orien",   Ktask_orien);
    node.getParam("gains/Djoint",        Djoint);
    node.getParam("gains/Kjoint",        Kjoint);

    // change this to your directory
    std::string urdf_file_name = filename;
    pinocchio::urdf::buildModel(urdf_file_name, model, false);				// read the URDF file
    data = 						pinocchio::Data(model);	
    dim_joints = model.nq ;

	// orientation finder
	pose_ref = 		 			pinocchio::SE3::Identity();					// reference pose
	pose_err_local = 			Eigen::VectorXd::Zero(6); 					// local position error in vector
	pose_err = 					Eigen::VectorXd::Zero(6); 					// position error in vector

    // initialize the joint positions
    x_final = 					Eigen::VectorXd::Zero(3);
    joint_pos = 				Eigen::VectorXd::Zero(dim_joints);
    joint_vel = 				Eigen::VectorXd::Zero(dim_joints);

    // Jacobian in the world frame 
    jacobian_local_world = 		Eigen::MatrixXd::Zero(6,dim_joints) ;
    jacobian_dot = 				Eigen::MatrixXd::Zero(6,dim_joints) ;
    jacobian_pseudo_inv = 		Eigen::MatrixXd::Zero(dim_joints,6) ;
    jacobian_trans_pseudo_inv = Eigen::MatrixXd::Zero(6,dim_joints) ;

    // Initialize end effector velocity and joint states 
    x_dot = 					Eigen::VectorXd::Zero(6);
    x_ref = 					Eigen::VectorXd::Zero(6);
    x_ref_dot = 				Eigen::VectorXd::Zero(6);
    x_ref_ddot = 				Eigen::VectorXd::Zero(6);

    q_ref = 					Eigen::VectorXd::Zero(dim_joints);
    q_ref_dot = 				Eigen::VectorXd::Zero(dim_joints);
    q_ref_ddot = 				Eigen::VectorXd::Zero(dim_joints);
    q_cmd_ddot = 				Eigen::VectorXd::Zero(dim_joints);

    // task space controller
    x_cmd_ddot = 				Eigen::VectorXd::Zero(6);
    n = 						Eigen::VectorXd::Zero(6);
    F_cmd = 					Eigen::VectorXd::Zero(6);
    T_task = 					Eigen::VectorXd::Zero(dim_joints);
    A = 						Eigen::MatrixXd::Zero(6,6);

	x_fbk = 					Eigen::VectorXd::Zero(6);
    x_fbk_dot = 				Eigen::VectorXd::Zero(6);

    // joint space controller
    D = 						Eigen::MatrixXd::Zero(6,6);
    P = 						Eigen::MatrixXd::Zero(dim_joints,dim_joints);
    T_joint = 					Eigen::VectorXd::Zero(dim_joints);
    T_null = 					Eigen::VectorXd::Zero(dim_joints);
    T = 						Eigen::VectorXd::Zero(dim_joints);

    // Choose the dt
    dt = 						Eigen::MatrixXd::Identity(dim_joints, dim_joints);
    dt = dt * 0.001;

    // publish to topic "/gen3/joint_group_position_controller/command"
    pose_pub = node.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command", 3);
	
	// subscribe to topic "/gazebo/model_states"
	pose_sub = node.subscribe("/gen3/joint_states", 1000, &Server::feedbackCallback, this);

	move_to_action_server.start();
	ROS_INFO_STREAM("[ActionServer::ActionServer] action server is ready");
};

// subscriber callback
void Server::feedbackCallback(sensor_msgs::JointState msg) {

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

void Server::dynamics_controller(){

	// ****************************************potential field planner*******************************************
	// **********************************************************************************************************
	// a5: orientation potential field planner																		
	pinocchio::SE3 pose_err_mat = data.oMi[JOINT_ID].inverse() * pose_ref;						// pose error
	pose_err_local = pinocchio::log6(pose_err_mat).toVector();									// position error in vector
	pose_err.tail(3) = data.oMi[JOINT_ID].rotation() * pose_err_local.tail(3);					// orientation error
	pose_err.head(3) = x_final - x_fbk.head(3);													// position error


	x_ref_dot = Katrr*(pose_err);
	// set limits on end effector velocity
	if (x_ref_dot.head(3).norm() > 0.5){
		x_ref_dot.head(3) = x_ref_dot.head(3)/x_ref_dot.head(3).norm() * 0.5;}
	x_ref = x_fbk + dt.block(0,0,6,6) * x_ref_dot;

	// FORWARD KINEMATICS
	pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);							// forward kinematics
	pinocchio::SE3 pose_now = data.oMi[JOINT_ID]  ;												// end-effector pose

	// get the current end effector position
	x_fbk.head(3) = pose_now.translation();
	x_fbk.tail(3) = pose_now.rotation().eulerAngles(0,1,2);




	// *****************************************task space controller*********************************************
	// ***********************************************************************************************************

	// Jacobian in the world frame 
	pinocchio::computeAllTerms(model, data, joint_pos, joint_vel);
	pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;

	// Jacobian dot in the world frame 
	pinocchio::computeJointJacobiansTimeVariation(model, data, joint_pos, joint_vel );
	pinocchio::getJointJacobianTimeVariation(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot) ;

	// Pseudo Inverse of Jacobian 
	jacobian_pseudo_inv = jacobian_local_world.completeOrthogonalDecomposition().pseudoInverse() ;
	jacobian_trans_pseudo_inv = (jacobian_local_world.transpose()).completeOrthogonalDecomposition().pseudoInverse() ;

	// get the current end effector velocity
	x_fbk_dot = jacobian_local_world * joint_vel;

	// Model parameters 
	M = data.M;
	h = data.nle;

	// Desired end effector torque 
	x_cmd_ddot.head(3) = x_ref_ddot.head(3) + Dtask_trans * (x_ref_dot.head(3) - x_fbk_dot.head(3)) + Ktask_trans * (x_ref.head(3) - x_fbk.head(3));
	x_cmd_ddot.tail(3) = x_ref_ddot.tail(3) + Dtask_orien * (x_ref_dot.tail(3) - x_fbk_dot.tail(3)) + Ktask_orien * (x_ref.tail(3) - x_fbk.tail(3));
	A = jacobian_trans_pseudo_inv * M * jacobian_pseudo_inv;
	n = jacobian_trans_pseudo_inv * (Eigen::MatrixXd::Identity(dim_joints,dim_joints) * h) - A * jacobian_dot * joint_vel;
	F_cmd = A * x_cmd_ddot + n;
	T_task = jacobian_local_world.transpose() * F_cmd;

	


	// ******************************************** joint space controller ***************************************
	// ***********************************************************************************************************
	KatrrM = Katrr * Eigen::MatrixXd::Identity(dim_joints,dim_joints);
	Eigen::VectorXd q_tar(dim_joints);
	q_tar << 0.0f, 0.0f, 0.0f,-1.57f, 0.0f, 0.75f, 1.57f;

	q_ref_dot = KatrrM *(q_tar - joint_pos);
	q_ref = joint_pos + dt * q_ref_dot;
	q_cmd_ddot = q_ref_ddot + Djoint * (q_ref_dot - joint_vel) + Kjoint * (q_ref - joint_pos);
	T_joint = M * q_cmd_ddot + (Eigen::MatrixXd::Identity(dim_joints,dim_joints) * h);

	// P 
	D = jacobian_local_world * M.inverse() * jacobian_local_world.transpose();
	P = Eigen::MatrixXd::Identity(dim_joints, dim_joints) - jacobian_local_world.transpose() \
	* D.inverse() * jacobian_local_world * M.inverse();

	// Torque
	T_null = P * T_joint;
	T = T_null + T_task;
	// **********************************************************************************************************
	// **********************************************************************************************************

	// joint torque limits
	T(0) = T(0) >  39.0f ?  39.0f : T(0);
	T(0) = T(0) < -39.0f ? -39.0f : T(0);
	T(1) = T(1) >  39.0f ?  39.0f : T(1);
	T(1) = T(1) < -39.0f ? -39.0f : T(1);
	T(2) = T(2) >  39.0f ?  39.0f : T(2);
	T(2) = T(2) < -39.0f ? -39.0f : T(2);
	T(3) = T(3) >  39.0f ?  39.0f : T(3);
	T(3) = T(3) < -39.0f ? -39.0f : T(3);
	T(4) = T(4) >  9.0f ?  9.0f : T(4);
	T(4) = T(4) < -9.0f ? -9.0f : T(4);
	T(5) = T(5) >  9.0f ?  9.0f : T(5);
	T(5) = T(5) < -9.0f ? -9.0f : T(5);
	T(6) = T(6) >  9.0f ?  9.0f : T(6);
	T(6) = T(6) < -9.0f ? -9.0f : T(6);

	// publish the joint positions
	pub_msg.data.clear();
	pub_msg.data.push_back(T(0));
	pub_msg.data.push_back(T(1));
	pub_msg.data.push_back(T(2));
	pub_msg.data.push_back(T(3));
	pub_msg.data.push_back(T(4));
	pub_msg.data.push_back(T(5));
	pub_msg.data.push_back(T(6));
	pose_pub.publish(pub_msg);

}

void Server::move_to_callback(const highlevel_controller::MoveToGoalConstPtr& goal) {

	ROS_INFO_STREAM("[ActionServer::move_to_callback] goal: x=" << goal->x << ", y=" << goal->y << ", z= " << goal->z) ;

	x_final(0) = goal->x;
	x_final(1) = goal->y;
	x_final(2) = goal->z;

	pose_err.head(3) = x_final - x_fbk.head(3);	

	while((pose_err.head(3)).norm() > 0.005f) {
		
		loopRate.sleep();
	}

	move_to_action_server.setSucceeded();
	ROS_INFO_STREAM("[ActionServer::move_to_ballback] Action is succesfully done") ;

}