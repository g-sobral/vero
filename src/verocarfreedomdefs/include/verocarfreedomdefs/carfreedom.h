/*
 * Orca-Robotics Project: Components for robotics
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2009 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This copy of Orca is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#ifndef HYDRO_INTERFACES_CARFREEDOM_H
#define HYDRO_INTERFACES_CARFREEDOM_H

#include "ros/ros.h"
#include <verocarfreedomdefs_msgs/CarData.h>
#include <verocarfreedomdefs_msgs/CarCommand.h>
//#include <gbxutilacfr/mathdefs.h> // min
#include <sockutil/dvsutil.h>

namespace verocarfreedom
{

/*!
    @ingroup hydro_drivers
    @defgroup hydro_drivers_carfreedom CarFreedom drivers
    @brief Implementations of CarFreedom hardware interface
*/
/*!
    @ingroup hydro_interfaces
    @defgroup hydro_interface_carfreedom CarFreedom
    @brief Access to CarFreedom robotic platform.

    @{
*/

/*!

@brief Abstract interface class for providing access to CarFreedom robotic platform.

Member functions throw Exception on error conditions.

The implementations of this interface are @b not expected to be thread-safe.

@author L Mirisola

*/
	std::string CarDatatoString(verocarfreedomdefs_msgs::CarData carData);
//! Motion command str virtual int decode(struct can_frame frame);
class Command
 {
 public:
   Command(float vl, float vr ,float ang) { carCommand.speedLeft=vl, carCommand.speedRight=vr; carCommand.steerAngle=ang;}
   Command() { carCommand.speedLeft=carCommand.speedRight=carCommand.steerAngle=0;}
   bool isZero() {return ((0==carCommand.speedLeft)&&(0==carCommand.speedRight)&&(0==carCommand.steerAngle)); }
   verocarfreedomdefs_msgs::CarCommand carCommand;
   //! Converts to a human readable string
   std::string toString() const;

 };
 //! Capabilities of the hardware
 class Capabilities
 {
   public:
     //! A positive quantity
double maxForwardSpeed;
     //! A positive quantity
     double maxReverseSpeed;
     //! left or right: one positive, other negative
     double maxSteeringAngleLeft;
//! left or right: one positive, other negative
double maxSteeringAngleRight;
     //! A positive quantity
     double maxSteeringAngleAtMaxSpeed;

     bool isSane() const;
     std::string toString() const;

 };

 struct joylimits {
   unsigned int maxforward;
   unsigned int maxreverse;
   unsigned int maxleft;
   unsigned int maxright;
 };

 struct Config {
   double wheel_radius_m; /// the wheel radius in [m]
   double inter_axis_m; /// distance between the front and rear axes in [m]
   double axis_length; /// the distance between the left and right wheels [m]
   double pulses_per_360; /// the number of encoder pulses in a complete 360o turn.
   unsigned int max_left_steering_encoder; ///the value of the steering encoder at max left turn
   unsigned int max_right_steering_encoder; ///the value of the steering encoder at max right turn
   struct joylimits joy; /// stores the extreme values of the joystick commands.
   bool encoder_frontleft_invert;/// 1 if negative encoder pulses means movement to front
   bool encoder_frontright_invert;/// 1 if negative encoder pulses means movement to front
   bool encoder_rearleft_invert;/// 1 if negative encoder pulses means movement to front
   bool encoder_rearright_invert;/// 1 if negative encoder pulses means movement to front
   int write_cmd_rph; ///1: rpm commands will be written; 0: joystick commands will be written ; 2: rpm commands written with differential values. Usually it will be 1
   double encoder_Vunc; ///the uncertainty in the velocity measured by the encoders, in m/s
 };
/*
 //! Initialises the hardware.
 //! Throws Exception on failure.
 //! It is not an error to call this repeatedly: repeated calls should
 //! re-initialise the hardware.
 //! (eg this will be called if the hardware reports an error condition)
 virtual void enable()=0;

 //! Blocking read.
 virtual void read( Data &data )=0;

 //! Writes motion command.
 virtual void write( const Command& command )=0;

 //! Get information about the current status of the driver.
 //! the string 'status' is a human-readable description.
 //! Note that there are two ways for faults to be notified:
 //!  - This function tells of hardware faults reported normally by the hardware.
 //!  - Exceptions can be thrown from read/write for non-recoverable faults, such as inability to
 //!    communicate with the hardware.
 virtual void getStatus( std::string &status, bool &isWarn, bool &isFault )=0;


 //! Read the capabilities of the hardware
 virtual Capabilities capabilities() const=0;
*/
//! Helper function for constraining capabilities based on some limits
//! (does a thresholding operation)
void constrain( Capabilities &capabilities,
                const Capabilities &limits );

/*! @} */
} // namespace
#endif
