#include "ros/ros.h"
#include <verocarfreedomdefs_msgs/CarData.h>
#include <verocarfreedomdefs/carfreedom.h>

using namespace verocarfreedom;

void PrettyPrint (verocarfreedomdefs_msgs::CarData msg){
	ROS_INFO("%s", CarDatatoString(msg).c_str());
} //PrettyPrint

int main (int argc, char **argv){
  // Inicia o modulo no ros.
  ros::init(argc, argv, "verocarfreedom_pretty_print");
  ros::NodeHandle n;


	ros::Subscriber car_data_sub = n.subscribe("car_data", 1, PrettyPrint);

	ros::spin ();
	return 0;
}

