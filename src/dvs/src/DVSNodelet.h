
#ifndef DVS_NODELET_H
#define DVS_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h> 
#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> // string stream
#include <nodelet/ESMTrack.h> /// our own message to report about tracking results
#include <esmocv/Dvs_Status.h> /// message to report dvs results. move here, right? 
#include "DvsCtrl.h"
#include "DDvsCtrl.h"
#include "RobotFactory.h"
#include <sockutil/dvsutil.h>
#include <sockutil/hzutil.h> // calculate fps...

namespace dvs
{
//class DVSNodelet;

/**
 * \brief abstract class to implement a nodelet with a control algorithm
 * \par it has most of the stuff, subclasses just need to implement the method which creates the appropriate algorithm.
 * \par each algorithm this defines a nodelet type for it.
 */
class CTRLNodelet : public nodelet::Nodelet
{
public:
  CTRLNodelet() ;

  ~CTRLNodelet();
  
  
protected:
  virtual void onInit();
  ros::Publisher dvs_pub; /// publishes control results
  ros::Subscriber trk_sub;/// needs tracking results  
  ros::NodeHandle nh_; /// global nodehandle for topics
  ros::NodeHandle nhPriv_;/// second, private nodehandle for parameters
  //image_transport::ImageTransport it_;
  image_transport::ImageTransport *itp_; /// it actually works as a pointer. 
  image_transport::Subscriber image_sub_; /// get images from camera
  //image_transport::Publisher image_pub_; /// uncomment if you want to publish
  //Ctrl *ctrl_;/// control algorithm
  boost::shared_ptr<Ctrl> ctrl_; ////// control algorithm
  cv::Rect ROI; /// tracked region on the image. 
  cv::Mat pt; /// the control point in the original image.
  cv::Mat G; /// the homography
  cv::Mat Gc; 
  cv::Mat G0;
  cv::Mat K; /// camera calibration matrix
  bool got_track; /// have we got tracking results yet?
  bool ctrl_success_;
  virtual Ctrl* createCtrl() = 0; /// abstract. subclasses must define their ctrl algoritms
  void ROIupdated() ;
  void fillBogusCameraInfo();
  boost::shared_ptr<RobotProxy> robot_;/// pointer to robot class. (created with parameter + factory)
  //RobotProxy *robot_; /// pointer to robot class. (created with parameter + factory)
  
  dvsutil::HZUtil fpsutil_; /// calculate fps and count cycles...
  void ImgCallback(const sensor_msgs::ImageConstPtr& msg);
  virtual void TrkCallback(const nodelet::ESMTrack& msg) ;
}; // class

/**
 * \brief nodelet for direct visual servoing
 */
class DVSNodelet : public CTRLNodelet {
  Ctrl* createCtrl () {
    /// MUST USE PRIVATE NODEHANDLE, TO READ PARAMETERS
    return new DvsCtrl(nhPriv_);
  }
}; /// class

/**
 * \brief nodelet for DECOUPLED direct visual servoing
 */
class DDVSNodelet : public CTRLNodelet {
  Ctrl* createCtrl () {
    /// MUST USE PRIVATE NODEHANDLE, TO READ PARAMETERS
    return new DDvsCtrl(nhPriv_);
  }
}; /// class


}//namespace


#endif 