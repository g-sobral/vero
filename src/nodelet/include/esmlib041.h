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
#include <sockutil/dvsutil.h>
#include <sockutil/hzutil.h>
#include <nodelet/ESMTrack.h> /// our own message to report about tracking results

/// ESMLib is different in the 32 and 64 bit versions. it must be compiled differently
#if defined(__i386__) || defined(__ILP32__)
  // IA-32
  extern "C" {
    #include "ESMlibry_040_ia32.h" 
    // Ezio Library v 0.4.0 for 32 bits - it has no mask functions! 
  }
#elif defined(__x86_64__) || defined(__LP64__)
  // AMD64
  extern "C" {
  #include "ESMlibry.h" 
  // Ezio Library v 0.4.1 for 64 bit - mask functions available
  }
#else
  #error unsupported architecture!
#endif

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
  double zncc_min; /// the min acceptable zncc value. If less, stop tracking
  double max_RMS; /// maximum acceptable RMS error, it greater, stop tracking
  int correlation_window; /// size of correlation window around ROI, if zero no correlation will be done
  
  ESMTrackParam() { // set default parameters
    miter=5; mprec=2;
    height = 480; width = 640;
    posx = 200; posy = 250;
    sizx = 100; sizy = 100;
    zncc_min = 0.9;
    max_RMS = 50.0;
    correlation_window=0;
  }
  
  void print() {
   std::cout << "ESMTrack Param iter " << miter << " mprec " << mprec << " pos [" << posx <<
   "," << posy << "] siz ["<< sizx <<","<<sizy<<"] wid " << width << " hei " << height << " zncc " << zncc_min << " RMSmx " << max_RMS << std::endl;
  }
  
}; // struct;

std::ostream& operator<<(std::ostream& out, const struct ESMTrackParam& p);

 /**
  * \brief C++ adapter for the ESMlib v. 0.4.1 64bit from Ezio
  * \par the lib and this class only perform tracking, the odometry lib is not covered, neither dvs or visual servoing
  */
class ESMlibTrack
{
public:
  ESMlibTrack()  {hasref=false; init();};
  ESMlibTrack(ESMTrackParam &tparam)  {tparam_=tparam;init();};
  void setParam(ESMTrackParam &tparam) {tparam_=tparam;init();};
  int setReferenceImageWithMask( const cv::Mat &image_ref, const cv::Mat &mask, 				bool saveIref);
  int setReferenceImage( const cv::Mat &image_ref , bool saveIref) ;
  int track( const cv::Mat &image , cv::Mat &H, nodelet::ESMTrack &trackstats, bool saveImg) ;
  

  ~ESMlibTrack() {
    if (use_mask_) {
#if defined(__LP64__)
      FreeTrackMask (&T);
#endif
    } else
      FreeTrack (&T);
    imgRef_.release();
    img_.release();
    //cv::destroyAllWindows(); /// kill all named windows.
  }// destructor
  
protected:
  nodelet::ESMTrack trackstats_; /// stores stats from last track
  int init() ;
  void correlate(const cv::Mat &newimage, bool corr_debug=false) ;
  float calcRMSerror();
  bool use_mask_ ; /// true, only the pixels marked by the mask are tracked; false, the whole ROI is tracked
  ESMTrackParam tparam_;
  
  /// The image read / acquired (ESM type)
  imageStruct Imask;
  /// The image read / acquired (ESM type)
  imageStruct I; 
  /// The global tracking structure (ESM type)
  trackStruct T; 
  
  cv::Mat imgRef_; /// the reference image 
  cv::Mat img_; /// ESMlib needs FLOAT images (32bit). If the input is not FLOAT, it will be converted and stored here in img_ as a FLOAT. 
  
  cv::Mat mask_; /// stores the mask which determines which pixels are tracked
  cv::Mat H_; /// stores the last computed homography in ocv format
  
  bool hasref;  /// do we have a reference Image set ? Cant do anything whithout one.
  
  void DrawLine (imageStruct *image, int r1, int c1, int r2, int c2);
  void DrawResult (int sx, int sy, float H[9], imageStruct I) ;
  void showIstructMask(imageStruct *im, std::string name);
  void showIstruct(imageStruct *im, std::string name);
  dvsutil::HZUtil fpsutil_;/// fps calculation and image counting
  dvsutil::HZUtil timeutil_;/// calculation of time to perform each step...
}; // class

}//namespace


#endif 
