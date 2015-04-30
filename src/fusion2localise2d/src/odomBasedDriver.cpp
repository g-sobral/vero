/*
 * Orca-Robotics Project: Components for robotics
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2009 Alex Brooks
 *
 * This copy of Orca is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */

#include <iostream>
#include <opencv/cv.h>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "odomBasedDriver.h"

using namespace std;

namespace fusion2localise2d {

OdometryBasedDriver::OdometryBasedDriver()
{
    prevTime_ = ros::Time(0, 0);

    // TODO: should make these configurable.
    minHeadingUncertainty_ = 10.0*M_PI/180.0;
    linSpeedThreshold_ = 0.75;
    linFactor_         = 30.0*M_PI/180.0;
    rotFactor_         = 180*M_PI/180.0;

    gps_heading_spd_threshold_ms_ = 0.5 / 3.6f; /// convert from km/h to m/s

    f = new Filter ();
}
/*
double
OdometryBasedDriver::calcHeadingUncertainty( hydronavutil::Pose &delta,
                                             double dt )
{
    // Uncertainty is low if we're moving fast in a straight line.
    double linSpeed = dt*hypot(delta.x(),delta.y());
    double rotSpeed = dt*delta.theta();

    double rotVal = fabs(rotSpeed);

    if ( linSpeed > linSpeedThreshold_ )
        linSpeed = linSpeedThreshold_;

    double linVal = linSpeedThreshold_-(linSpeed);

    double uncertainty =
        minHeadingUncertainty_  +
        rotFactor_ * rotVal     +
        linFactor_ * linVal;

//     cout<<"TRACE(odometrybaseddriver.cpp): -----------------" << endl;
//     cout<<"TRACE(odometrybaseddriver.cpp): linSpeed: " << linSpeed << endl;
//     cout<<"TRACE(odometrybaseddriver.cpp): rotSpeed: " << rotSpeed*180.0/M_PI << "deg/s" << endl;
//     cout<<"TRACE(odometrybaseddriver.cpp): rotFactor_*rotVal = (" << rotFactor_ << "*" << rotVal << ")="<<rotFactor_*rotVal * 180.0/M_PI << "deg" << endl;
//     cout<<"TRACE(odometrybaseddriver.cpp): linFactor_*linVal = (" << linFactor_ << "*" << linVal << ")="<<linFactor_*linVal * 180.0/M_PI << "deg" << endl;
//     cout<<"TRACE(odometrybaseddriver.cpp): uncertainty: " << uncertainty*180.0/M_PI << "deg" << endl;


    return uncertainty;
}*/


/**
 * \brief with insData, it uses the compass for heading, but only when the vehicle is stopped
 * \li calculates speed from odometry
 * \li if speed < gps_heading_spd_threshold_ms_ then it ignores gps heading and uses the compass
 * \li else, it ignores the compass and uses gps heading
 * \li the xy coordinates are just gps coordinates.
 */


bool OdometryBasedDriver::compute( nav_msgs::Odometry *odomGps,
				   nav_msgs::Odometry  *odomRoda,
				   geometry_msgs::PoseWithCovariance *localiseData)
{    

 	f->pred_ukf(odomRoda->pose.pose.position.x,odomRoda->pose.pose.position.y,tf::getYaw(odomRoda->pose.pose.orientation));
	if (odomGps != NULL) f->updateWithGPS(odomGps->pose.pose.position.x,odomGps->pose.pose.position.y,tf::getYaw(odomGps->pose.pose.orientation));
	//else f->updateWithoutGPS();
	

    if (f->isInit()){

	 localiseData->pose.position.x 				= f->getFusionX();
	 localiseData->pose.position.y 				= f->getFusionY();
	 localiseData->pose.position.z 				= 0;
	 tf::quaternionTFToMsg (tf::createQuaternionFromYaw(f->getFusionO()), localiseData->pose.orientation);

	 localiseData->covariance[0] = f->getFusionxx();
	 localiseData->covariance[7] = f->getFusionyy();
	 localiseData->covariance[1] = localiseData->covariance[6] = f->getFusionxy();

	 localiseData->covariance[5] = localiseData->covariance[30] = f->getFusionxt();
	 localiseData->covariance[11] = localiseData->covariance[31] = f->getFusionyt();
	 localiseData->covariance[35] = f->getFusiontt();

    }
    else return false;

    if (odomGps != NULL)prevTime_ = odomGps->header.stamp;
    return true;
}//compute


}//namespace
