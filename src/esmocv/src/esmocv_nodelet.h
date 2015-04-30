/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ESMOCV_NODELET_H
#define ESMOCV_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h> 
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream> // string stream
#include "esmocv.h"
#include "esmocv/SetROI.h" // service to restart tracking

namespace esmocv
{
class ESMOCVNodelet;

class ESMOCVNodelet : public nodelet::Nodelet
{
public:
  ESMOCVNodelet() ;

  ~ESMOCVNodelet() ;
  
  const char* WINDOW;/// used for debug. esm now handles that.
private:
  virtual void onInit();
  ros::Publisher dvs_pub;  
  ros::Publisher trk_pub;  
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_;// second, private nodehandle
  boost::shared_ptr<image_transport::ImageTransport> it_; 
  
  int waitForRestartROI_; /// do we start at the beggining or wait until the service is called?
  
  image_transport::Subscriber image_sub_; // get images from camera
  image_transport::Publisher imageSlow_pub_; // uncomment if you want to publish
  ros::ServiceServer servSetROI;
  void ImgCallback(const sensor_msgs::ImageConstPtr& msg);
  bool restartROI(esmocv::SetROI::Request& req,esmocv::SetROI::Response& ans);
  
  boost::shared_ptr<ESMOCV> esm_;  /// work horse
  bool initHasRun; /// need to know if init has ended.
  void printMat(string msg,const Mat &M);
  
  /// used to publish
  std::string slower_image_topic_;
  int pub_every_frame_;
  int count_frames_;
  
  /// thread management
  sensor_msgs::ImageConstPtr msgIn2; ///consumer stores pointer to received image, freeing msgIn_ to be reused
  sensor_msgs::ImageConstPtr msgIn_; ///producer stores pointer to received image
  boost::shared_ptr<boost::thread > procThread_ ;   
  bool procThread_running_; /// true if procThread_ is running
  void RawImgCallback(const sensor_msgs::ImageConstPtr& msg);
  void ProcImgThread(void);
  boost::mutex mutex_full; 
  boost::mutex mutex_empty;
  boost::mutex mutex_shared;
  // for mutexes see
  //http://en.wikipedia.org/wiki/Producer%E2%80%93consumer_problem
  //http://en.wikipedia.org/wiki/Semaphore_%28programming%29#Semaphore_vs._mutex
  
  
}; // class

}//namespace


#endif 