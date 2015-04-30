#include "sockutil/hzutil.h"

using namespace dvsutil;

/**
 * \par calculate the duration since the last call.
 * @return false at the first call, in this case period will not be written; true at the next calls.
 */
bool HZUtil::cycle(ros::Duration &period){
  cycleCount_++;
  ros::Time current = ros::Time::now();
  if (!valid_) { // no data to calculate at the first call. 
    tstamp_ = current; // save for the next time
    valid_ = true; // next time we'll have data...
    return false;
  }
  period = current - tstamp_; // return the last period
  tstamp_ = current; // save for the next time
  return true;
}//cycle


/**
 * \par Convenience function. Calculate the duration since the last call and return the time in seconds as a float
 * @return false at the first call, in this case period will not be written; true at the next calls.
 */
bool HZUtil::cycleSec(float &period) {
  ros::Duration per;
  bool ret = cycle(per);
  if (ret) period = per.toSec();
  return ret;
};
/**
 * \par Convenience function. calculate the duration since the last call and return the frequency in HZ (like fps) as a float
 * @return false at the first call, in this case period will not be written; true at the next calls.
 */
bool HZUtil::cycleHZ(float &hz) {
  ros::Duration per;
  bool ret = cycle(per);
  if (ret) hz = 1/per.toSec();
  return ret;
};

/**
 * \par Convenience function. Do not Calculate anything, just start the time counting. it still returns false at the first call.
 * @return false at the first call; true at the next calls.
 */
bool HZUtil::cycle() {
  ros::Duration per;
  bool ret = cycle(per);
  return ret;
};
