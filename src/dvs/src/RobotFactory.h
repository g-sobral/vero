#ifndef DVS_ROBOT_FACTORY_H
#define DVS_ROBOT_FACTORY_H
#include "robot.h"
#include "StaubliRobot.h"
#include "PioneerRobot.h"

namespace dvs
{
/**
 * \brief factory method implemented as a singleton. Ask the singleton to create a robotproxy for you. 
 */
class RobotFactory {
private:
  static RobotFactory* instance;
protected:
  RobotFactory(){}
public:
  
  static RobotFactory* uniqueInstance() {
    if (NULL==instance) instance = new RobotFactory();
    return instance;
  }//uniqueInstance
  /// if you have a new subclass, define here when it is created.
  RobotProxy* newRobot(std::string &robotname, ros::NodeHandle &nh) ;
  
}; //class

}//namespace
#endif