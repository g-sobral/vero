#include "ransac_lib.h"

Publisher *pub;
double vero_lenght, pioneer_lenght;

void pioneerCallback(const ransac_project::CarCommand& msg)
{
	geometry_msgs::Twist msg_pub;
	double omega;
	
	omega = msg.speedLeft / vero_lenght * tan(msg.steerAngle);
	
	msg_pub.linear.x = msg.speedLeft;	/*speedLeft = speedRight*/
	msg_pub.linear.y = 0;
	msg_pub.linear.z = 0;
	msg_pub.angular.x = 0;
	msg_pub.angular.y = 0;
	msg_pub.angular.z = atan(omega * pioneer_lenght / msg.speedLeft);
	
	pub->publish(msg_pub);
}

void ctrlHandler(int x)
{
	geometry_msgs::Twist msg_pub;
	
	msg_pub.linear.x = 0;
	msg_pub.linear.y = 0;
	msg_pub.linear.z = 0;
	msg_pub.angular.x = 0;
	msg_pub.angular.y = 0;
	msg_pub.angular.z = 0;
	
	pub->publish(msg_pub);
	
	sleep(1);
	exit(0);
}

int main(int argc, char** argv)
{
	init(argc, argv, "vero2pioneer");
	NodeHandle node;
	NodeHandle node_aux("~");
	
	if(!node_aux.getParam("vero_lenght", vero_lenght) || !node_aux.getParam("pioneer_lenght", pioneer_lenght)){
		ROS_ERROR("parameters not specified");
		exit(0);
	}
	
	signal(SIGINT, ctrlHandler);
	
	pub = new Publisher();
	*pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	Subscriber sub = node.subscribe("car_command", 1, pioneerCallback);
	spin();
	
	return 0;
}
