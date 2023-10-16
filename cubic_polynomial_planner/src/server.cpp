#include "ros/ros.h"
#include "cubic_polynomial_planner/move_robot.h"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/duration.h>

class Server
{
  // init the node
  public:

	// target pose
    double xf, yf, zf;

	// current pose
	double xs, ys, zs;

	// update pose
	double xt, yt, zt;

	bool flag = false;

	// pose messages
	gazebo_msgs::ModelStates msg;
	geometry_msgs::PoseStamped pub_msg;

	// to record the time
	ros::Time ros_start_time;
	ros::Duration total_service_time;
	ros::Time ros_end_time;

	// functions
	bool service_callback( cubic_polynomial_planner::move_robot::Request  &req,
						   cubic_polynomial_planner::move_robot::Response &res);
	void subscriber_callback(gazebo_msgs::ModelStates msg);
	void update_pose();

	ros::NodeHandle n;
	// constructor
	Server(ros::NodeHandle &node) {
		n = node;
	}
};



// service callback
bool Server::service_callback( cubic_polynomial_planner::move_robot::Request  &req,
						   cubic_polynomial_planner::move_robot::Response &res) {
	
	// get target pose
	xf = (double) req.x;
	yf = (double) req.y;
	zf = (double) req.z;

	// get the start time, end time and total service time
	ros_start_time = ros::Time::now();
	total_service_time = ros::Duration((double) req.T);
	ros_end_time = ros_start_time + total_service_time;
	
	// set flag to true once service is called
	res.flag = flag = true;
	return true;
}

// subscriber callback
void Server::subscriber_callback(gazebo_msgs::ModelStates msg) {
	// save global model state
	xs = msg.pose[1].position.x;
	ys = msg.pose[1].position.y;
	zs = msg.pose[1].position.z;
}

// update pose
void Server::update_pose(){
	
	// publish to topic "/firefly/command/pose"
  	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);

	while(ros::Time::now() < ros_end_time) {

		// update the time reemaining
		ros::Duration duration = ros::Time::now() - ros_start_time;

		// compute timescale for third order polynomial
		double s = 3/(total_service_time.toSec()*total_service_time.toSec()) * duration.toSec() * duration.toSec() \
					- 2/(total_service_time.toSec()*total_service_time.toSec()*total_service_time.toSec()) \
					* duration.toSec() * duration.toSec() * duration.toSec();

		// compute the updated pose
		xt = xs + s * (xf - xs);
		yt = ys + s * (yf - ys);
		zt = zs + s * (zf - zs);

		// update pose messages
		pub_msg.pose.position.x = xt;
		pub_msg.pose.position.y = yt;
		pub_msg.pose.position.z = zt;
		
		// publish the pose
		pose_pub.publish(pub_msg);

		// indicate when target position is reached
		if (abs(ros::Time::now().toSec() - ros_end_time.toSec()) < 0.001) {
			ROS_INFO("Reached target position");
			break;
		}
	}	
}



int main(int argc, char **argv) {

	// initialize node
	ros::init(argc, argv, "server");

	// create node handle
	ros::NodeHandle node;

	// specify the frequency to 10HZ
	ros::Rate loopRate(100) ;

	// advertise the service
	Server server(node);

	// advertise a service "/cubic_polynomial_planner/move_robot"
	ros::ServiceServer service = node.advertiseService("/cubic_polynomial_planner/move_robot", &Server::service_callback, &server);
	
	// subscribe to topic "/gazebo/model_states"
	ros::Subscriber pose_sub = node.subscribe("/gazebo/model_states", 1, &Server::subscriber_callback, &server);

	// // publish to topic "/firefly/command/pose"
  	// ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);

	// Serive move_robot is created
	ROS_INFO("Server: a service to move the robot is created");

	// ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while ( ros::ok() ) {
	
	// if service is called
	if (server.flag) {
		
		// update the pose
		server.update_pose();
	}

    // spinOnce() will call all the callbacks waiting to be called at that point in time
	ros::spinOnce();

	// sleep for any time remaining to the publish rate
	loopRate.sleep();

	}
  return 0;
}