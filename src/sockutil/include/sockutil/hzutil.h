#ifndef DVSUTIL_HZUTIL_H
#define DVSUTIL_HZUTIL_H

#include <math.h>       /* round, floor, ceil, trunc */
#include <ros/ros.h>

namespace dvsutil {

/**
 * \brief Utility class to compute fps of images, or periods of period events. 
 * \par any of methods cycle* returns the time since the last call of any of the cycle* methods. There are various flavors, just to allow various different output formats.
 * \par it also counts how many times the cycle* methods were called.  
 */
class HZUtil {
public:
  HZUtil( ) : valid_(false),cycleCount_(0) {};
  bool cycle();
  bool cycle(ros::Duration &period);
  bool cycleSec(float &period);
  bool cycleHZ(float &hz) ;
  long getCount() { return cycleCount_;}
  void reset() { cycleCount_=0; valid_=false;}
   
private:
  ros::Time tstamp_;
  bool valid_; /// valid only if we have already got the first timestamp
  long cycleCount_; /// how many times cycle was called.
  
};//class

}//namespace

#endif