
#ifndef ESMOCV_ROBOT_H
#define ESMOCV_ROBOT_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>       /* round, floor, ceil, trunc, M_PI, sin,cos */
#include <opencv2/core/core.hpp>
#include <esmocv/Dvs_Status.h> /// our own message to report about dvs results
#include <sstream> // string stream
#include <sockutil/dvsutil.h> // convertMat2boost::array
#include <sockutil/sockclient.h> // staublisocket

using namespace cv;

namespace esmocv
{

/**
 * \brief abstract class to implement communication and frame transforms for the robot.
 */
class RobotProxy {
public:
  /// nodelets need to be initialized to get a private nodehandle. therefore in a nodelet you should create a pointer to this class and pass the private nodehandle in the constructor.
  RobotProxy(ros::NodeHandle &nh) : nh_(nh) {};

  ~RobotProxy(){};
  
  /// send command to robot
  virtual int commRobot(esmocv::Dvs_Status &dvsstat)=0;
  /// stops the robot
  virtual int stopRobot(esmocv::Dvs_Status &dvsstat)=0;
  
protected:
  
  /// transforms velocities from camera to robot frame.
  virtual int transCam2Rob(esmocv::Dvs_Status &dvsstat)=0;

    
  ///////// initialization stuff
  ros::NodeHandle nh_;/// needed to get the parameters   
};//class RobotProxy

};//namespace

#endif
