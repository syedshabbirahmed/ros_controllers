#include "inverse_dynamics_controller/Server.hpp"

Server::Server(){

    node.getParam("/gen3/urdf_file_name",filename);
    node.getParam("gains/Katrr", Katrr);
    node.getParam("gains/Dtask", Dtask);
    node.getParam("gains/Ktask", Ktask);
    node.getParam("gains/Djoint", Djoint);
    node.getParam("gains/Kjoint", Kjoint);

    // change this to your directory
    std::string urdf_file_name = filename;
    pinocchio::urdf::buildModel(urdf_file_name, model, false);				// read the URDF file
    data = pinocchio::Data(model);	
    dim_joints = model.nq ;

    // initialize the joint positions
    x_final = Eigen::VectorXd::Zero(3);
    joint_pos = Eigen::VectorXd::Zero(dim_joints);
    joint_vel = Eigen::VectorXd::Zero(dim_joints);

    // Jacobian in the world frame 
    jacobian_local_world = Eigen::MatrixXd::Zero(6,dim_joints) ;
    jacobian_dot = Eigen::MatrixXd::Zero(6,dim_joints) ;
    jacobian_pseudo_inv = Eigen::MatrixXd::Zero(dim_joints,3) ;
    jacobian_trans_pseudo_inv = Eigen::MatrixXd::Zero(3,dim_joints) ;

    // Initialize end effector velocity and joint states 
    x_dot = Eigen::VectorXd::Zero(3);
    x_ref = Eigen::VectorXd::Zero(3);
    x_ref_dot = Eigen::VectorXd::Zero(3);
    x_ref_ddot = Eigen::VectorXd::Zero(3);

    q_ref = Eigen::VectorXd::Zero(dim_joints);
    q_ref_dot = Eigen::VectorXd::Zero(dim_joints);
    q_ref_ddot = Eigen::VectorXd::Zero(dim_joints);
    q_cmd_ddot = Eigen::VectorXd::Zero(dim_joints);

    // task space controller
    x_cmd_ddot = Eigen::VectorXd::Zero(3);
    n = Eigen::VectorXd::Zero(3);
    F_cmd = Eigen::VectorXd::Zero(3);
    T_task = Eigen::VectorXd::Zero(dim_joints);
    A = Eigen::MatrixXd::Zero(3,3);

    x_fbk = Eigen::VectorXd::Zero(3);
    x_fbk_dot = Eigen::VectorXd::Zero(3);

    // joint space controller
    D = Eigen::MatrixXd::Zero(3,3);
    P = Eigen::MatrixXd::Zero(dim_joints,dim_joints);
    T_joint = Eigen::VectorXd::Zero(dim_joints);
    T_null = Eigen::VectorXd::Zero(dim_joints);
    T = Eigen::VectorXd::Zero(dim_joints);

    // Choose the dt
    dt = Eigen::MatrixXd::Identity(dim_joints, dim_joints);
    dt = dt * 0.001;

    // publish to topic "/gen3/joint_group_position_controller/command"
    pose_pub = node.advertise<std_msgs::Float64MultiArray>("/gen3/joint_group_effort_controller/command", 3);
};

// service callback
bool Server::service_callback( inverse_dynamics_controller::move_robot::Request  &req,
						   inverse_dynamics_controller::move_robot::Response &res) {
	
	// get target pose
	x_final(0) = (long double) req.x;
	x_final(1) = (long double) req.y;
	x_final(2) = (long double) req.z;

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
	x_fbk = pose_now.translation().head(3);
	
	flag = false;
	pub_flag = true;
}


void Server::dynamics_controller(){

	// ****************************************potential field planner*******************************************
	// **********************************************************************************************************
	x_ref_dot = Katrr*(x_final - x_fbk);
	x_ref_dot = x_ref_dot.norm() > 0.5 ? x_ref_dot / x_ref_dot.norm() * 0.5 : x_ref_dot;
	x_ref = x_fbk + dt.block(0,0,3,3)*x_ref_dot;

	// FORWARD KINEMATICS
	pinocchio::forwardKinematics(model, data, joint_pos, joint_vel);							// forward kinematics
	pinocchio::SE3 pose_now = data.oMi[JOINT_ID]  ;												// end-effector pose

	// get the current end effector position
	x_fbk = pose_now.translation().head(3);



	// *****************************************task space controller*********************************************
	// ***********************************************************************************************************

	// Jacobian in the world frame 
	pinocchio::computeAllTerms(model, data, joint_pos, joint_vel);
	pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;

	// Jacobian dot in the world frame 
	pinocchio::computeJointJacobiansTimeVariation(model, data, joint_pos, joint_vel );
	pinocchio::getJointJacobianTimeVariation(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot) ;

	// Pseudo Inverse of Jacobian 
	jacobian_pseudo_inv = jacobian_local_world.block(0,0,3,dim_joints).completeOrthogonalDecomposition().pseudoInverse() ;
	jacobian_trans_pseudo_inv = (jacobian_local_world.block(0,0,3,dim_joints).transpose()).completeOrthogonalDecomposition().pseudoInverse() ;

	// get the current end effector velocity
	x_fbk_dot = jacobian_local_world.block(0,0,3,dim_joints) * joint_vel;

	// Model parameters 
	M = data.M;
	h = data.nle;

	// Desired end effector torque 
	x_cmd_ddot = x_ref_ddot + Dtask * (x_ref_dot - x_fbk_dot) + Ktask * (x_ref - x_fbk);
	A = jacobian_trans_pseudo_inv * M * jacobian_pseudo_inv;
	n = jacobian_trans_pseudo_inv * (Eigen::MatrixXd::Identity(dim_joints,dim_joints) * h) - A * jacobian_dot.block(0,0,3,dim_joints) * joint_vel;
	F_cmd = A * x_cmd_ddot + n;
	T_task = jacobian_local_world.block(0,0,3,dim_joints).transpose() * F_cmd;

	


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
	D = jacobian_local_world.block(0,0,3,dim_joints) * M.inverse() * jacobian_local_world.block(0,0,3,dim_joints).transpose();
	P = Eigen::MatrixXd::Identity(dim_joints, dim_joints) - jacobian_local_world.block(0,0,3,dim_joints).transpose() \
	* D.inverse() * jacobian_local_world.block(0,0,3,dim_joints) * M.inverse();

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