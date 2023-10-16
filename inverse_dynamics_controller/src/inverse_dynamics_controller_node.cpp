#include <pinocchio/algorithm/compute-all-terms.hpp>
#include "inverse_dynamics_controller/Server.hpp"

int main(int argc, char **argv) {

	// initialize node
	ros::init(argc, argv, "inverse_dynamics_controller_node");

	// create node handle
	ros::NodeHandle node;

	// specify the frequency to 100HZ
	ros::Rate loopRate(1000) ;

	// create server class
	Server server;

	// advertise a service "/cubic_polynomial_planner/move_robot"
	ros::ServiceServer service = node.advertiseService("/planner/move_to", &Server::service_callback, &server);
	
	// subscribe to topic "/gazebo/model_states"
	ros::Subscriber pose_sub = node.subscribe("/gen3/joint_states", 1000, &Server::subscriber_callback, &server);

	// Serive move_robot is created
	ROS_INFO("Server: a service to move the robot is created");
	
	// ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while(ros::ok()){

		ros::spinOnce(); // will call all the callbacks waiting to be called at that point in time

				// if service is called
		if (server.flag) {
			// find the initial robot pose
			server.initial_pose();
		}

		// if service is called
		if (server.pub_flag) {

			// update the pose
			server.dynamics_controller();

		}

		loopRate.sleep();
	}

	return 0 ;
}