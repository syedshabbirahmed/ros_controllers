#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using namespace std;


class Listener
{
  // init the node
  public:
    string key; 
    geometry_msgs::Twist geom_msg;

  // functions
  void call_back_function ( const std_msgs::String::ConstPtr& msg);
  void update_twist();
};

void Listener::call_back_function ( const std_msgs::String::ConstPtr& msg) {
      
      // Access Data
      key = msg -> data;
    }

void Listener::update_twist() {

      // Update Twist
      if (key == "i") {
        geom_msg.linear.x = 1;
        geom_msg.angular.z = 0;
      }
      else if (key == "o") {
        geom_msg.linear.x = 1;
        geom_msg.angular.z = 1;
      }
      else if (key == "u") {
        geom_msg.linear.x = 1;
        geom_msg.angular.z = -1;
      }
      else {
        geom_msg.linear.x = 0;
        geom_msg.angular.z = 0;
      }
  }

int main(int argc, char **argv)
{
  // initialize ROS
  ros::init(argc, argv, "husky_teleop_controller_node");

  // create a node handle
  ros::NodeHandle node;

  // specify the frequency to 10HZ
	ros::Rate loopRate(10) ;

  // create a subscriber object
  Listener listener;

  // listening to topic "teleop/cmd" 
  ros::Subscriber sub = node.subscribe("teleop/cmd", 1, &Listener::call_back_function, &listener);

  // publish to topic "husky_velocity_controller/cmd_vel"
  ros::Publisher vel_pub = node.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);

  // ros::ok() returns false when the program is finished (e.g., when you do Ctrl-C)
	while ( ros::ok() ) {
    
    // Update the velocity
    listener.update_twist();

    // Publish the velocity
    vel_pub.publish(listener.geom_msg);

    // reset the key
    listener.key = "reset";

    // spinOnce() will call all the callbacks waiting to be called at that point in time
		ros::spinOnce() ;

		// sleep for any time remaining to the publish rate
		loopRate.sleep() ;
	}
  return 0;
}