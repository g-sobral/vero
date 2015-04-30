/*
 * Orca-Robotics Project: Components for robotics
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2009 Alex Brooks, Alexei Makarenko, Tobias Kaupp
 *
 * This distribution is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <sstream>
#include <iostream> // cerr
#include <iomanip> // setw
#include <verocarfreedomdefs/carfreedom.h>
#include <assert.h>
//#include <gbxutilacfr/mathdefs.h>
#include <sockutil/dvsutil.h>
#include <cmath> //fabs

static const double Pi = 3.14159265358979323846264338327950288419717;

namespace verocarfreedom {

std::string CarDatatoString(verocarfreedomdefs_msgs::CarData carData)
{
    std::stringstream ss;
    bool modeerror = true;
    int width=9;

    // ------------- printing bicData.
    //ss << bicData.toString();// we could use its own pretty print routine
    // but here it is a better print out
    ss << "  Time(s):    " << carData.timeStamp.sec << "s"<<std::setw(6)<< carData.timeStamp.nsec << "us"  << std::endl;
    ss << "  x,y,yaw:          [" << std::setw(width) << carData.x << " " << carData.y << " " << RAD2DEG(carData.yaw) << "]" << std::endl;
    ss << "  steerAng (deg): " << RAD2DEG(carData.steerAngle) << std::endl;
    ss << "  vlong: m/s " << carData.vlong    << std::endl;
    ss << "  dyaw: (deg/s) " << RAD2DEG(carData.dyaw)  << std::endl;
    //---------- finished printing bicData
    ss << "  Speed REAR L " << std::setw(width) << carData.speedLeft << " R " << carData.speedRight << std::endl;
    ss << "  Speed FRNT L " << std::setw(width) << carData.speedLeftFront << " R " << carData.speedRightFront << std::endl;
    // enc_missing is char, so you need to cast to (int) in order to print it out.
    ss << "  Missing EncFrames:     " << (int)carData.enc_missing[0] << " " << (int)carData.enc_missing[1] << " " << (int)carData.enc_missing[2] << " " << (int)carData.enc_missing[3] << std::endl;

    ss << "  mode: ";
    if (1==carData.mode) { ss <<" REMOTE" ; modeerror = false; }
    if (2==carData.mode) { ss <<" LOCAL" ;modeerror = false; }
    if (3==carData.mode) { ss <<" AUTONOMOUS" ;modeerror = false; }
    if (modeerror) ss << "ERROR: mode = " << carData.mode ;
    ss << std::endl;
    ss << "  Keys: " << (carData.CarOn?" ON ":"OFF ") << (carData.lights?"LIGHT ":"light ") <<
    (carData.buzz?"BUZZ ":"buzz ") << (carData.strobo?"STRB ":"strb ") << (carData.brake?"BRK ":"brk ")<< std::endl;;
    ss << "  Bump " << (carData.bumperFront?" F ":" f ") << (carData.bumperRear?" R ":" r ")<< std::endl;
    ss << "  Bat (V):    " << carData.battVolt    << std::endl;
    ss << "  Motor(A): L " << std::setw(width) << carData.ampereMotorLeft << " R " << carData.ampereMotorRight   << std::endl;
    ss << "  Motor acel: L " << std::setw(width) << carData.acelLeft << " R " << carData.acelRight << std::endl;
    ss << "  Steering " << (carData.endCourseSteering?" END ":" OK ") << " I:  " << carData.ampereSteering << std::endl;
    if (carData.hasWarnings ) ss << "WARN";
    if ( carData.hasWarnings && carData.hasFaults ) ss << "/";
    if ( carData.hasFaults ) ss << "FAULT";
    if ( carData.hasWarnings || carData.hasFaults )
        ss << ": " << carData.warnFaultReason << std::endl;
    return ss.str();
}

std::string Command::toString() const
{
  std::stringstream ss;
  ss << "  SteerAng :  " << RAD2DEG(carCommand.steerAngle) << std::endl;
  ss << "  SpeedL:" << carCommand.speedLeft << std::endl;
  ss << "  SpeedR:" << carCommand.speedRight << std::endl;
  return ss.str();
}

/**
 * \par forces steerAngle left to be negative and right to be positive
 */
bool Capabilities::isSane() const
{
    return ( maxForwardSpeed >= 0.0 &&
             maxReverseSpeed >= 0.0 &&
             maxSteeringAngleLeft <= 0.0 &&
	     maxSteeringAngleRight >= 0.0 &&
             maxSteeringAngleAtMaxSpeed >= 0.0 &&
	     maxSteeringAngleAtMaxSpeed <= MIN(fabs(maxSteeringAngleLeft),fabs(maxSteeringAngleRight)) );
}//isSane

std::string Capabilities::toString() const
{
    std::stringstream ss;
    ss << std::endl
       << "  maxForwardSpeed: " << maxForwardSpeed << "m/s" << std::endl
       << "  maxReverseSpeed: " << maxReverseSpeed << "m/s" << std::endl
       << "  maxSteeringAngleLeft:     " << maxSteeringAngleLeft*180.0/M_PI << "deg" << std::endl
       << "  maxSteeringAngleRight:     " << maxSteeringAngleRight*180.0/M_PI << "deg" << std::endl
       << "  maxLateralAcceleration: " << maxSteeringAngleAtMaxSpeed*180.0/M_PI << "deg";
    return ss.str();
}//toString

/**
 * \par it really suposes that left steerAngle is negative
 */
void constrain( Capabilities &capabilities,
           const Capabilities &limits )
{
    assert( capabilities.isSane() );
    assert( limits.isSane() );

    capabilities.maxForwardSpeed = MIN( capabilities.maxForwardSpeed, limits.maxForwardSpeed );
    capabilities.maxReverseSpeed = MIN( capabilities.maxReverseSpeed, limits.maxReverseSpeed );
    capabilities.maxSteeringAngleLeft = MAX(capabilities.maxSteeringAngleLeft,limits.maxSteeringAngleLeft);
    capabilities.maxSteeringAngleRight = MIN( capabilities.maxSteeringAngleRight,limits.maxSteeringAngleRight );
    capabilities.maxSteeringAngleAtMaxSpeed = MIN( capabilities.maxSteeringAngleAtMaxSpeed, limits.maxSteeringAngleAtMaxSpeed );
}

} // namespace
