
#ifndef DVS_STAUBLIROBOT_H
#define DVS_STAUBLIROBOT_H


#include "robot.h"

namespace dvs {

class StaubliRobot : public RobotProxy {
public:
  StaubliRobot(ros::NodeHandle &nh); 
  ~StaubliRobot() { if(sock_) delete sock_; };
  
  
  int commRobot(esmocv::Dvs_Status &dvsstat);
  int stopRobot(esmocv::Dvs_Status &dvsstat);

  
protected:
  
  int transCam2Rob(esmocv::Dvs_Status &dvsstat);
  
  
  int readParam();

  sockutil::StaubliClient *sock_;

  double theta_;
  Mat Tc_;
};//class 

};//namespace

#endif