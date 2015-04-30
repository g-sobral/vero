#include "RobotFactory.h"

 
namespace dvs
{

  RobotFactory* RobotFactory::instance=NULL;/// just initialization 
  
  /// if you have a new subclass, define here when it is created.
  RobotProxy* RobotFactory::newRobot(std::string &robotname, ros::NodeHandle &nh) {
    if ((robotname == "staubli") ) {
        return new StaubliRobot(nh);
    }
    if ((robotname == "pioneer") ) {
        return new PioneerRobot(nh); 
    }
    ROS_ERROR("RobotFactory:: unknown robot name: %s", robotname.c_str());
    return NULL;
  } 
  
}//namespace  