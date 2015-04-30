
#ifndef ESMOCV_MAIN_H
#define ESMOCV_MAIN_H

#include <ros/ros.h>
#include <stdio.h>
#include <math.h>       /* round, floor, ceil, trunc */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <esmlib041.h>
//#include <esmocv/Dvs_Status.h> /// our own message to report about dvs results
//#include "CtrlFactory.h"
//#include <sockutil/sockclient.h>
//#include <sockutil/dvsutil.h>
//#include "RobotFactory.h"
#include "mask.h"

using namespace cv;

namespace esmocv
{
/**
 * \par DO NOT HANDLE ROS COMMUNICATION HERE. DO IT IN THE NODELET CLASS.
 */
class ESMOCV;

struct ESMParam {
  double alpha;
  double beta;
  int maxIte;
  int iter_max;
  
  ESMParam() {
   alpha = 1;
   beta=0;
   maxIte = 30;
   iter_max = 1000;
  }
  
  void print() {
      ROS_INFO("ESMParam: maxIte %d alpha %.2f beta %.2f itermx %d",
	maxIte,alpha,beta,iter_max
      );
  }//print
  
};//ESMParam struct

/**
 * \brief higher level class for ESM tracking and DVS control. 
 * \par RATIONALE it should be easy to use this class outside of the nodelet.  
 */
class ESMOCV 
{
public:
  ESMOCV(ros::NodeHandle &nh);

  ~ESMOCV();
  
  void setROI(Rect &newR);
  
  void procImg(const Mat &I, Mat &H_, nodelet::ESMTrack &trackmsg);
  void setIref(const Mat &I);
  void closeOCVWindowOnDelete(bool b) { closeWindowOnDelete=b; } /// to faster restart, leave ocvwindows open
  const char* WINDOW; /// opencv named windows need a name
  const char* REFWINDOW; /// opencv named windows need a name
  Mat getSmallImage();
private:
  Rect ROI;/// the target region on the image
  Mat G0;
  Mat Iref;
  bool hasIref;/// has the reference image been set?
  Mat recDim;
  Mat recCur;
  Mat Ic;
  Mat patr;
  Mat K;
  Mat G;
  Mat Gc;
  
  int plot_every_frame;
  
  Mat ptw;
  Mat pt;
  float fps_cur;
  double focal_length;
  
  ESMParam p;/// from the adapter lib for esm
  int imageWidth;
  int imageHeight;
  
  ///////// initialization stuff
  ros::NodeHandle nh_;
  void fillBogusCameraInfo();
  void fillDefaults();
  cv::Rect readROIParams() ;

  ////// the core tracking variables
  esmlib::ESMlibTrack track; /// struct from Ezio lib for tracking
  esmlib::ESMTrackParam tparam;/// there parameters come from Ezio Lib
  
  //////// variables needed to draw results
  long frameCount;
  Mat Ismall; /// store a reduced image for displaying
  void DrawResult (Mat &I, double scale) ;
  void DrawLine( Mat &img, Point start, Point end );
  double viewscale; /// the scale of the window which shows the tracking. no effect on the actual algorithm
  bool closeWindowOnDelete;
  
  //////////// mask issues
  int stepmask_; /// how many pixs to skip when using the stepmaskcreator
  
  ///////////// safety
  bool stop_; /// if true the show stops. 
  

}; // class

}//namespace



#endif 
