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

#ifndef ESMLIB_WRAPPER_H
#define ESMLIB_WRAPPER_H

//#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
//#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h> 
//#include <math.h> //fabs
#include <boost/shared_ptr.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

extern "C" {
#include "ESMlibry.h" // Ezio Library
}

namespace esmlib
{

struct ESMTrackParam {
  
  int            miter; //!< iteration number 
  int            mprec; //!< tracking precision 
  int posx;
  int posy;
  int sizx;
  int sizy;
  int width ; 
  int height ; 
  
  ESMTrackParam() { // set default parameters
    miter=5; mprec=2;
    posx = 200; posy = 250;
    sizx = 100; sizy = 100;
    height = 480; width = 640;
  }
  
}; // struct;
  
  
class ESMlibTrack
{
public:
  ESMlibTrack()  {init();};
  ESMlibTrack(ESMTrackParam &tparam)  {tparam_=tparam;init();};
  void setParam(ESMTrackParam &tparam) {tparam_=tparam;init();};
  
  int setReferenceImage( const cv::Mat &image_ref , bool saveIref) ;
  int track( const cv::Mat &image , cv::Mat &H, bool saveImg) ;
  

  ~ESMlibTrack() {
    FreeTrack (&T);
    imgRef_.release();
    img_.release();
  }// destructor
  
private:
  
  int init() ;
  
  ESMTrackParam tparam_;
  
  
  
   // The image read / acquired (ESM type)
  imageStruct I; 
  // The global tracking structure (ESM type)
  trackStruct T; 
  
  cv::Mat imgRef_;
  cv::Mat img_;
  int countimg_;
  bool hasref;
  
  void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2);
  void DrawResult (int sx, int sy, float H[9], imageStruct I) ;
  
}; // class

}//namespace


#endif 