
#ifndef DVS_ABSTRACTCTRL_H
#define DVS_ABSTRACTCTRL_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>       /* round, floor, ceil, trunc, PI */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <esmocv/Dvs_Status.h> /// our own message to report about dvs results
#include <sstream> // string stream
#include <sockutil/dvsutil.h> // convertMat2boost::array
#include <cmath> // exp 

using namespace cv;

namespace dvs
{

  
struct CtrlParam {
  double lambda;
  double eps; /// limit on the error norm, if smaller, stop the robot
  bool stop_is_definitive; /// when stop condition is reached, it will remain stopped
  double adapt_gamma; /// adaptive gain exponential decay constant (if zero, (constant) gain = lambda)
  
  CtrlParam() {
   lambda = 0.3;
   eps = 1e-3;
   stop_is_definitive=1;
   adapt_gamma = 0;
  }
  
  void print() {
      ROS_INFO("CtrlParam: l %.3f gam %.3f eps %.3g stopdef %s",
	lambda, adapt_gamma,eps,(stop_is_definitive?"T":"F")
      );
  }//print
  
  
};//CtrlParam struct

std::ostream& operator<<(std::ostream &out, const struct CtrlParam &c);
  
  
  
/**
 * \brief abstract class to implement control algorithms
 */
class Ctrl
{
public:
  /// needs nodehandle in constructor to read parameters
  Ctrl(ros::NodeHandle &nh) : nh_(nh) {readParam(); has_stopped=false;};

  ~Ctrl(){}
  
  // matlab line [v_c(:,iter),e(:,iter)] = dvsPlanar(Gc, K, lambda, pt);
  virtual int doCtrl(const Mat &G, const Mat &K, const Mat &pt, esmocv::Dvs_Status &dvsstat)=0;
  
  virtual bool stopCondReached() { 
    bool condition = ( cv::norm(e_) < param.eps );  
    if (!param.stop_is_definitive) return condition;
    if (condition) has_stopped = true;
    return has_stopped;
  }
  
  virtual int readParam(); 
  
protected:
  double adapt_gain();
  
  // matlab function [axe_rot,angle_rot] = axe_ang(H)
  void axe_ang(const Mat &H, Mat &axe_rot, double &angle_rot);

  Mat e_;/// the error.
  
  
  ///////// initialization stuff
  ros::NodeHandle nh_;/// needed to get the parameters 
  CtrlParam param;
  bool has_stopped;
}; // class
  
 

}//namespace



#endif 