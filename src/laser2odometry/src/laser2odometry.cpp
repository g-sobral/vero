#include "laser2odometry_classes.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "laser2odometry");
	laser2odometry::SubscribeAndPublish laser2odom;
	
	ros::spin();

	return 0;
}
