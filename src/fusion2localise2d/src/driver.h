/*
 * Orca-Robotics Project: Components for robotics
 *               http://orca-robotics.sf.net/
 * Copyright (c) 2004-2009 Alex Brooks
 *
 * This copy of Orca is licensed to you under the terms described in
 * the LICENSE file included in this distribution.
 *
 */
#ifndef DRIVER_H
#define DRIVER_H

#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

namespace fusion2localise2d {

//
// @author Alex Brooks
//
class Driver
{

public:

    virtual ~Driver() {}

    // Converts the GPS info into localise2d format.
    // Returns 'false' if the conversion can't be done (eg because GPS doesn't have a fix).
    virtual bool compute( nav_msgs::Odometry *odomGps,
				   nav_msgs::Odometry  *odomRoda,
				   geometry_msgs::PoseWithCovariance *localiseData) = 0;
private:


};

}

#endif
