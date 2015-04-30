#include "verocarfreedom_test_node.h"

int main (int argc, char ** argv){
	verocarfreedom::Context car_context;
	socketcan_lib::Context socket_context;
	verocarfreedomdefs_msgs::CarCommand comando;
	verocarfreedomdefs_msgs::CarData data;

	comando.speedLeft = 1.9;
	comando.speedRight = 3.2;
	comando.steerAngle = 3.2;

  // Inicia o modulo no ros.
  ros::init(argc, argv, "verocarfreedom_test_node");
  ros::NodeHandle n;

	verocarfreedom::IniciaContext (car_context);
	socketcan_lib::IniciaContext (socket_context);
	
ROS_INFO("iniciado. baudrade:%d", socket_context.baudRate);

	verocarfreedom::Driver driver = verocarfreedom::Driver (car_context, socket_context);

ros::Rate cycle(100);

	while(ros::ok()) {
		ROS_INFO("iniciado..");
		driver.write (comando);
		//driver.read (data);
    ros::spinOnce();
	cycle.sleep();
	}

	return 0;
}
