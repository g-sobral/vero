
#ifndef DVS_PIONEERROBOT_H
#define DVS_PIONEERROBOT_H


#include "robot.h"

#include <tf/transform_datatypes.h>/// quaternion-RPY conversion
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace dvs {

class PioneerRobot : public RobotProxy {
public:
  PioneerRobot(ros::NodeHandle &nh); 
  ~PioneerRobot() { };
  
  
  int commRobot(esmocv::Dvs_Status &dvsstat);
  int stopRobot(esmocv::Dvs_Status &dvsstat);

  void pantiltPoseCallback(const geometry_msgs::Pose pos);

protected:
  
  int transCam2Rob(esmocv::Dvs_Status &dvsstat);
  
  
  int readParam();

  double lambda_w_; /// gain to rotation. 
  
  /// geometry variables
  double theta_;
  Mat rTc_;///translation camera->robot
  
  /// publishers and subscribers to communicate with robot AND pantilt
  ros::Publisher vel_pub_ ;
  ros::Publisher ptl_pub_ ;
  ros::Subscriber ptlpose_sub_ ; 
  
  geometry_msgs::Pose lastptl_; /// stores last pantilt pose (pan and tilt)
  bool has_ptl_pose_; /// have we received info about the pantilt pose?
  double robot_v_linear_ ; /// stores the calculated command to be sent
  double robot_w_angular_ ;/// stores the calculated command to be sent
  double pan_speed_ ;/// stores the calculated command to be sent
  double tilt_speed_ ;/// stores the calculated command to be sent
  
};//class 

};//namespace

#endif