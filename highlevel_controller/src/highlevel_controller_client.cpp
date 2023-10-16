#include <pinocchio/algorithm/compute-all-terms.hpp>
#include "highlevel_controller/ActionClient.hpp"

int main(int argc, char **argv) {

	// initialize node
	ros::init(argc, argv, "highlevel_controller_client");

	// create node handle
	ros::NodeHandle node;

	// specify the frequency to 100HZ
	ros::Rate loopRate(1000) ;

	ActionClient client(node);

	// Serive move_robot is created
	ROS_INFO("A node to move the robot is created");
	
	// ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while(ros::ok()){

		ros::spinOnce(); // will call all the callbacks waiting to be called at that point in time

		client.update();

		loopRate.sleep();
	}

	return 0 ;
}