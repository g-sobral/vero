#include <iostream>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include "odomBasedDriver.h"
#include "driver.h"

#define NODE_NAME "f2l"

using namespace std;
using namespace fusion2localise2d;

char* gps_topic_odom = (char *)"/gps_node/odom";
char* roda_topic_odom = (char *)"/verocarfreedom/roda_odom";

ros::Publisher pose_pub;
geometry_msgs::PoseWithCovariance *odom_msg;
OdometryBasedDriver *driver_;
nav_msgs::Odometry gps_msg, roda_msg;
bool gps_msg_updated = false;

void recuperaOdomMessageGps(const nav_msgs::Odometry::ConstPtr& event)
{
	gps_msg_updated = true;
  gps_msg = *event;
}

void recuperaOdomMessageRoda(const nav_msgs::Odometry::ConstPtr& event)
{
 	nav_msgs::Odometry newPose;

	roda_msg = *event;

	if (gps_msg_updated) {
		gps_msg_updated = false;
		driver_->compute(&gps_msg, &roda_msg, odom_msg);
	}
	else driver_->compute(NULL, &roda_msg, odom_msg);
	
	newPose.header.stamp = ros::Time::now();
	newPose.pose = *odom_msg;
	
	pose_pub.publish (newPose);
}

void displayHelp() 
{
  std::cerr << "fusion2localise2d\n"
            << std::endl
            << "Usage: rosrun fusion2localise2d fusion2localise2d_node <options>\n"
            << std::endl
            << "Options:\n"
            << "\t -h, -?       print usage message\n"
            << "\t -r <topic>   set roda_topic_odom topic (default: roda_node/odom)\n"
            << "\t -g <topic>   set gps_topic_odom topic (default: gps_node/odom)\n"
            << std::endl
            << "Example:\n"
            << "  rosrun fusion2localise2d fusion2localise2d_node -r odom -g gps/odom\n"
            << std::endl;
}


/** get command line and ROS parameters
 *
 * \returns 1 if successful
 */
int getParameters(int argc, char *argv[])
{

  // use getopt to parse the flags
  char ch;
  const char* optflags = "h:r:g:?";
  while(-1 != (ch = getopt(argc, argv, optflags)))
    {
      switch(ch)
        {
        case 'r':
          //roda_topic_odom = optarg;
          break;
        case 'g':
          //gps_topic_odom = optarg;
			 break;
        default:                        // unknown
          ROS_WARN("unknown parameter: %c", ch);
          // fall through to display help...
        case 'h':                       // help
        case '?':
          displayHelp();
          return 0;
        }
    }

  return 1;
}

int main (int argc, char **argv){

	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	
	pose_pub = n.advertise<nav_msgs::Odometry>("pose2d", 100);

	odom_msg = new geometry_msgs::PoseWithCovariance();
	driver_ = new OdometryBasedDriver();
	if (!getParameters (argc, argv)) return 0;

	// Inscreve no tópico de odom da roda.
	ros::Subscriber roda_sub = n.subscribe(roda_topic_odom, 100, recuperaOdomMessageRoda);
	// Increve no tópico odom do gps.
	ros::Subscriber gps_sub = n.subscribe(gps_topic_odom, 100, recuperaOdomMessageGps);
	
	ros::spin();	

	return 0;
}
