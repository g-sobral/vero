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

#include "pgr_simple_nodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(ptgrey_simple, PgrSimpleNodelet, ptgrey_simple::PgrSimpleNodelet, nodelet::Nodelet);


namespace ptgrey_simple
{

PgrSimpleWorker* PgrSimpleWorker::instance=NULL;  
  
void PgrSimpleNodelet::onInit()
  {
    ros::NodeHandle private_nh = getPrivateNodeHandle();
    image_transport::ImageTransport it(private_nh);
    pub = it.advertise("pgr_mono", 1);
    PgrSimpleWorker* worker = PgrSimpleWorker::uniqueInstance(this); 
    NODELET_INFO("onInit");
    //private_nh.getParam("value", value_);
     
    // imageptr is a boost shared pointer, this is the recipe to initialize it
    image_msg = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
    image_msg->data.resize(1280*960);
    
    frameCount=0;
    printIntervalSec = 10;
    fps = 0;
    
    gcbc.InitPointGrey( );
    gcbc.RunSingleCamera(&PgrSimpleNodelet::wrapper_callback);
   
  }//onInit
  
  
void PgrSimpleNodelet::callback(Image* pImage, const void* pCallbackData) {
  frameCount++;  
  // at the first frame, save its time and count
  if (1 == frameCount) { lastTStamp = ros::Time::now(); lastFrameCount = 1; }
  ros::Duration elapsed = ros::Time::now() - lastTStamp;
  if ( elapsed.toSec() > printIntervalSec ) {
    fps = (frameCount - lastFrameCount) / elapsed.toSec();
    lastFrameCount = frameCount;
    lastTStamp = ros::Time::now();
  }
  
  NODELET_INFO_THROTTLE(printIntervalSec,"point gray callback: %d frames fps %.2f",frameCount,fps);
    if ( pImage->GetTimeStamp().seconds != 0 ) {
      image_msg->header.stamp.sec  = pImage->GetTimeStamp().seconds;
      image_msg->header.stamp.nsec = pImage->GetTimeStamp().microSeconds*1e3;
    } else {
      // THROTTLE prints message every X seconds (first argument)
      NODELET_WARN_THROTTLE(printIntervalSec,"Pgr pImage has zeroed timestamp. Using ros::Time:Now()");
      image_msg->header.stamp = ros::Time::now();
    }
    image_msg->height = pImage->GetRows();
    image_msg->width = pImage->GetCols();
    // http://docs.ros.org/electric/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
    image_msg->encoding = sensor_msgs::image_encodings::MONO8; //sensor_msgs::image_encodings::MONO16;
    image_msg->step = image_msg->width ;
    if (image_msg->data.size() != pImage->GetDataSize() ) {
      NODELET_WARN("Resizing. Change default size!");
      image_msg->data.resize(pImage->GetDataSize());
    }
    
    /*/ allocates new buffer
    std::vector<uint8_t> newbuff(pImage->GetDataSize());
    // exchanges old buffer for new buffer
    image_msg->data.swap(newbuff);
    // when newbuff dies (when the method ends) the old buffer will be deallocated
    */
    // peform deep copy. 
    memcpy(&image_msg->data[0], pImage->GetData(), pImage->GetDataSize());
    // can not convert to a float image, because image_msg->data is uint8
    // can not share data because image_msg is a std::vector, and it can not be created around a existing buffer without hard copying.
    
    //sensor_msgs::CameraInfoPtr cinfo(new sensor_msgs::CameraInfo);
    //cinfo->header = image_msg->header;
    //cinfo->height = image_msg->height;
    //cinfo->width = image_msg->width;
    image_msg->header.seq = frameCount;
    pub.publish(image_msg);
    
    /*  format of the image ros message
    $ rosmsg show sensor_msgs/Image 
    Header header
    uint32 seq
    time stamp
    string frame_id
    uint32 height
    uint32 width
    string encoding
    uint8 is_bigendian
    uint32 step
    uint8[] data
    */
}// callback
  
//void operator PgrSimpleNodelet::()(Image* pImage, const void* pCallbackData) {
//  printf("hello i am the new callback");
//  
//}//callback

}//namespace


