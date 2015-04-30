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

#ifndef PGR_SIMPLE_NODELET_H
#define PGR_SIMPLE_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h> 
#include <math.h> //fabs
#include <boost/shared_ptr.hpp>

#include "GrabCallbackClass.h"

namespace ptgrey_simple
{
class PgrSimpleNodelet;

/**
 * \brief a singleton to hold a unique pointer to the PgrSimpleNodelet 
 * \par problem: PGR flycap lib only has a C callback function. we need a static function.
 * \par and the static function needs a pointer to a instance, otherwise everything would have to be static.
 * \par PgrSimpleNodelet can not be a singleton, as it is already a nodelet.
 * \par so this is not nice but was a way to make it work and to avoid a global variable
 */
class PgrSimpleWorker {
private:
  static PgrSimpleWorker* instance;
  PgrSimpleNodelet *mynodelet_;
protected:
  PgrSimpleWorker(PgrSimpleNodelet *mynodelet):mynodelet_(mynodelet){}
public:
  /**
   * if you are sure the singleton has already been created, you may call with a NULL argument
   */
  static PgrSimpleWorker* uniqueInstance(PgrSimpleNodelet *mynodelet) {
    if (NULL==instance) instance = new PgrSimpleWorker(mynodelet);
    return instance;
  }//uniqueInstance
  PgrSimpleNodelet* getNodelet() { return mynodelet_; } 
  
  
}; //class
  
class PgrSimpleNodelet : public nodelet::Nodelet
{
public:
  PgrSimpleNodelet()  {}

  ~PgrSimpleNodelet() {
    gcbc.StopSingleCamera( );  
  }// destructor
  
  void callback(Image* pImage, const void* pCallbackData);
  
  void static wrapper_callback(Image* pImage, const void* pCallbackData)
  //void operator()(Image* pImage, const void* pCallbackData)
  {
    //printf("nodelet static callback");
    PgrSimpleWorker* worker = PgrSimpleWorker::uniqueInstance(NULL);
    ROS_ASSERT(worker);
    PgrSimpleNodelet* mynodelet = worker->getNodelet();
    if (mynodelet)
      mynodelet->callback(pImage, pCallbackData);
  }
  
private:
  virtual void onInit();
    
  GrabCallbackClass gcbc;
  
  // variables to calculate frame rate
  ros::Time lastTStamp;
  unsigned int lastFrameCount;
  unsigned int frameCount;
  double fps;
  int printIntervalSec; /// print status message every X seconds
  
  //image_transport::ImageTransport* it;
  image_transport::Publisher pub;
  sensor_msgs::ImagePtr image_msg;
  
}; // class

}//namespace


#endif 