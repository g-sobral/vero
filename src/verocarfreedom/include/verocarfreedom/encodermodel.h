
#ifndef HYDRO_CARFREEDOM_ENCODERMODEL_H
#define HYDRO_CARFREEDOM_ENCODERMODEL_H

#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <verocarfreedomdefs_msgs/CarData.h>
#include <verocarfreedomdefs_msgs/CarCommand.h>
#include <verocarfreedomdefs/carfreedom.h>
#include <candriver.h>
#include <verocarfreedomdefs/carfreedom_defs.h>

//#include <gbxutilacfr/mathdefs.h> // CLIP_TO_LIMITS, DEG2RAD
#include <sockutil/dvsutil.h>
#include "driver.h"

using namespace std;

namespace verocarfreedom
{

enum WHEEL {
    FRONT_LEFT=0,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT
};

class EncodersModel
{
  public:
    EncodersModel(verocarfreedom::Config &cfg);
    void new_encoder_reading(WHEEL w,unsigned int pulses,unsigned char counter, unsigned char errcode);
    void new_steeringAngle(double stAng) {stAng_=stAng;} /// saves the steering Angle (it has already been decoded)
    void update_carfreedomdata(verocarfreedomdefs_msgs::CarData *data_, double dt);
  private:
    void updateOdometry() ;
    void absolute2delta(WHEEL i,unsigned int pulses, unsigned char encoder_counter,unsigned char encoder_error);
    void check_encoder_counters(WHEEL i,unsigned char encoder_counter,unsigned char encoder_error) ;
    unsigned char last_encoder_counters[4]; /// keeps the state of the last values of the encoders frame counters
    unsigned int last_encoder_pulses[4]; /// keeps the last values of the absolute pulse counters.
    unsigned char missingframes[4]; /// unreported missing frames.
    long deltapulses[4]; /// unreported relative pulse counter. it should go negative, so it is signed
    bool initialized[4]; /// have we received the first data packet from the encoder?
    verocarfreedom::Config cfg_;
    geometry_msgs::Pose2D odom_;/// stores the current position of the car in the local frame. it is the odometry value reported.
    double Dth_; /// store the current angular displacement.
    double Ddt_; /// store the current linear displacement
    double stAng_;

};//class

} //namespace


#endif
