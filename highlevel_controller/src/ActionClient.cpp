#include "highlevel_controller/ActionClient.hpp"

ActionClient::ActionClient(ros::NodeHandle& node_handle): node(node_handle),
												   		  move_to_action_client("/move_to_action",true),
														  gripper_action_client("/gen3/finger_group_action_controller/gripper_cmd",true){

	target_translation	= Eigen::MatrixXd::Zero(NUM_TARGETS ,3) ;
	gripper_position	= Eigen::MatrixXd::Zero(NUM_GRIPPER_POSITIONS ,2) ;

	readParameters();

	ROS_INFO_STREAM("Successfully launched client node.");
}

bool ActionClient::readParameters() {

	node.getParam("home/x", 				target_translation(0,0)) ;
	node.getParam("home/y", 				target_translation(0,1)) ;
	node.getParam("home/z", 				target_translation(0,2)) ;
	node.getParam("default_position/x", 	target_translation(1,0)) ;
	node.getParam("default_position/y", 	target_translation(1,1)) ;
	node.getParam("default_position/z", 	target_translation(1,2)) ;
	node.getParam("pregrasp_position/x", 	target_translation(2,0)) ;
	node.getParam("pregrasp_position/y", 	target_translation(2,1)) ;
	node.getParam("pregrasp_position/z", 	target_translation(2,2)) ;
	node.getParam("release_position/x", 	target_translation(3,0)) ;
	node.getParam("release_position/y",	 	target_translation(3,1)) ;
	node.getParam("release_position/z", 	target_translation(3,2)) ;

	node.getParam("open_gripper/position"  , 	gripper_position(0,0)) ;
	node.getParam("open_gripper/max_effort", 	gripper_position(0,1)) ;
	node.getParam("grasp/position"  , 			gripper_position(1,0)) ;
	node.getParam("grasp/max_effort", 			gripper_position(1,1)) ;
	node.getParam("release/position"  , 		gripper_position(2,0)) ;
	node.getParam("release/max_effort", 		gripper_position(2,1)) ;
	return true;
}

void ActionClient::update() {
	
	GRASP_STATE current_state = HOME;

	switch (current_state) {
		case GRASP_STATE::HOME:

			// HOME
			action_goal.x = target_translation(0, 0);
			action_goal.y = target_translation(0, 1);
			action_goal.z = target_translation(0, 2);

			// wait for the server
			move_to_action_client.waitForServer();
			move_to_action_client.sendGoal(action_goal);

			ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
			move_to_action_client.waitForResult(ros::Duration(7.0));

		if (move_to_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
			current_state = DEFAULT_POSITION;
			}

		case GRASP_STATE::DEFAULT_POSITION:

			// DEFAULT_POSITION
			action_goal.x = target_translation(1, 0);
			action_goal.y = target_translation(1, 1);
			action_goal.z = target_translation(1, 2);

			// wait for the server
			move_to_action_client.waitForServer();
			move_to_action_client.sendGoal(action_goal);

			ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
			move_to_action_client.waitForResult(ros::Duration(7.0));

		if (move_to_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
			current_state = OPEN_GRIPPER ;
			}

		case GRASP_STATE::OPEN_GRIPPER:

			// OPEN_GRIPPER
			squeeze_goal.command.position   = gripper_position(0, 0);
			squeeze_goal.command.max_effort = gripper_position(0, 1);

			// wait for the server
			gripper_action_client.waitForServer();
			gripper_action_client.sendGoal(squeeze_goal);

			ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
			gripper_action_client.waitForResult(ros::Duration(7.0));

		if (gripper_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The gripper is open.");
			current_state = PREGRASP_POSITION;
			}

		case GRASP_STATE::PREGRASP_POSITION:

			// REACH
			action_goal.x = target_translation(2, 0);
			action_goal.y = target_translation(2, 1);
			action_goal.z = target_translation(2, 2);

			// wait for the server
			move_to_action_client.waitForServer();
			move_to_action_client.sendGoal(action_goal);

			ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
			move_to_action_client.waitForResult(ros::Duration(7.0));

		if (move_to_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
			current_state = GRASP;
			}
		
		case GRASP_STATE::GRASP:

			// OPEN_GRIPPER
			squeeze_goal.command.position   = gripper_position(1, 0);
			squeeze_goal.command.max_effort = gripper_position(1, 1);

			// wait for the server
			gripper_action_client.waitForServer();
			gripper_action_client.sendGoal(squeeze_goal);

			ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
			gripper_action_client.waitForResult(ros::Duration(7.0));

		if (gripper_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The gripper caught the object.");
			current_state = RELEASE_POSITION;
			}

		case GRASP_STATE::RELEASE_POSITION:

			// REACH
			action_goal.x = target_translation(3, 0);
			action_goal.y = target_translation(3, 1);
			action_goal.z = target_translation(3, 2);

			// wait for the server
			move_to_action_client.waitForServer();
			move_to_action_client.sendGoal(action_goal);

			ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
			move_to_action_client.waitForResult(ros::Duration(7.0));

		if (move_to_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The end-effector reaches the goal position") ;
			current_state = RELEASE;
			}
		
		case GRASP_STATE::RELEASE:

			// OPEN_GRIPPER
			squeeze_goal.command.position   = gripper_position(2, 0);
			squeeze_goal.command.max_effort = gripper_position(2, 1);

			// wait for the server
			gripper_action_client.waitForServer();
			gripper_action_client.sendGoal(squeeze_goal);

			ROS_INFO_STREAM("[ActionClient::update] Sent action goal. Waiting for the results...") ;
			gripper_action_client.waitForResult(ros::Duration(7.0));

		if (gripper_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO_STREAM("[ActionClient::update] Yay! The gripper dropped the object in the bowl.");
			break;
			}
	}

	ROS_INFO_STREAM("[ActionClient::doneCallback] Done, shotdown") ;
	ros::shutdown();
}