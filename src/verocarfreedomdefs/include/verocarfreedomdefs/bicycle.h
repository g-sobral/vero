#ifndef HYDRO_INTERFACES_BICYCLE_H
#define HYDRO_INTERFACES_BICYCLE_H

#include "ros/ros.h"
#include "ros/time.h"

namespace verocarfreedom{

//! Bicycle data: commanded and actual steering angle & velocity.
struct DriveBicycleData
{
   //! Time when data was measured.
   ros::Time timeStamp;

   //! Reference value for the angle of the steered wheel [rad].
   //! This is the command currently executed by the vehicle.
   double referenceSteerAngle;

   //! Reference value for the forward speed of the steered wheel [m/s].
   //! The value is negative when the vehicle is to move backwards.
   double referenceSpeed;

   //! Current angle of the steered wheel [rad]
   double currentSteerAngle;

   //! Current forward speed of the steered wheel [m/s].
   //! The value is negative when the vehicle moves backwards.
   double currentSpeed;

	int steerAngle;
};

}//namespace
#endif
