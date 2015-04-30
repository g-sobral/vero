#include "ros/ros.h"
#include <verocarfreedomdefs/driver.h>
#include <verocarfreedomdefs_msgs/CarData.h>
#include "verocarfreedomdefs/carfreedom.h"

using namespace verocarfreedom;

void PrettyPrint (verocarfreedomdefs_msgs::CarData msg){
	ROS_INFO("%s", CarDatatoString(msg).c_str());
} //PrettyPrint

int main (int argc, char **argv){
  // Inicia o modulo no ros.
  ros::init(argc, argv, "verocarfreedom_pretty_print");
  ros::NodeHandle n;

	unsigned int nframes;
	verocarfreedomdefs_msgs::CarData data;
	socketcan_lib::Context context;
	socketcan_lib::IniciaContext (context);

	verocanprint::Driver *driver = new verocanprint::Driver (context);

	ros::Rate cycle(100);

	while(ros::ok()) {
		//ROS_INFO("iniciado..");
		nframes = 1;
		driver->readData (nframes);
		cycle.sleep();
		ros::spinOnce();
	}
	return 0;
}
