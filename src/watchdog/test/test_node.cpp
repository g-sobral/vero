#include <stdio.h>
#include "ros/ros.h"
#include "watchdog/watchdog.hpp"

#define CYCLE 100
#define DURATION 0.1

int main (int argc, char **argv){
  ros::init(argc, argv, "watchdog_test_node");
  ros::NodeHandle n;
  WatchDog* watchdog;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate cycle(CYCLE);

  watchdog = new WatchDog(n);

  watchdog->StartTimer(DURATION);

  while (ros::ok()){
    ROS_INFO ("I am alive!");
	 watchdog->IsAlive();
	 cycle.sleep();
    ros::spinOnce();
  }
	return 0;
}
