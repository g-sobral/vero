/*
 * Orca-Robotics Project: Components for robotics
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2009 Alex Brooks
 *
 * This copy of Orca is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef ODOMETRY_BASED_DRIVER_H
#define ODOMETRY_BASED_DRIVER_H

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "filter.h"
#include "driver.h"


namespace fusion2localise2d {

//
// @brief Assumes that the GPS unit works out heading by differentiating.
//
//        Works out heading uncertainty  based on this assumption, plus the odometric velocity
//        of the platform.
//
// @author Alex Brooks
//
class OdometryBasedDriver : public Driver
{

public:

    OdometryBasedDriver();

    // Converts the GPS info into localise2d format.
    // Returns 'false' if the conversion can't be done (eg because GPS doesn't have a fix).
    bool compute( nav_msgs::Odometry *odomGps,
				   nav_msgs::Odometry  *odomRoda,
				   geometry_msgs::PoseWithCovariance *localiseData);

private:

    //double calcHeadingUncertainty( hydronavutil::Pose &delta,
      //                             double dt );


    //hydronavutil::OdometryDifferentiator         odometryDifferentiator_;

    ros::Time prevTime_;

    double minHeadingUncertainty_;
    double linSpeedThreshold_;
    double rotFactor_;
    double linFactor_;
    double gps_heading_spd_threshold_ms_;/// slower: use heading from compass. faster: use gps heading
    //hydronavutil::Pose delta_; /// stores the delta xy-th from the last odometry pose to the current one

    Filter *f;
};

}

#endif
