#include "laser2odometry_classes.h"

//using namespace laser2odometry;

// Construtor
laser2odometry::SubscribeAndPublish::SubscribeAndPublish() {
	  //Topic you want to publish
		  pub_1 = n_.advertise<nav_msgs::Odometry>("laser_odom", 1);
	
	  //Topic you want to subscribe
	  sub_1 = n_.subscribe("laser_data", 1, &SubscribeAndPublish::callback_laser, this);
	  sub_2 = n_.subscribe("roda_odom", 1, &SubscribeAndPublish::callback_odom, this);
	  
	  has_odom = false;
	  odoLaser = new laserodometry::TelemetricOdometry();
	  
	  contador = 0;
}


void laser2odometry::SubscribeAndPublish::incrementa_contador () {
	if (contador == DIV_FREQ_HOKUYO)
		contador = 0;
	else
		contador ++;
}


void laser2odometry::SubscribeAndPublish::callback_odom (const nav_msgs::Odometry& odom) {
	

	has_odom = true;
	odometryData = odom;

}

/* Salva os dados do laser */
void laser2odometry::SubscribeAndPublish::callback_laser (const sensor_msgs::LaserScan& laserData) {

	
	// Put the range data in the struct             
 	for(int i = 0; i < N; i++) {
		ls.r[i] = (double)laserData.ranges[i];
	}
	
	if (has_odom) {
		
		if (contador == 5) {
		double theta = tf::getYaw(odometryData.pose.pose.orientation);
		hokuyo_relative_to_base(theta);
		
		ls.x = -odometryData.pose.pose.position.y;
		ls.y = odometryData.pose.pose.position.x;
		ls.theta = tf::getYaw(odometryData.pose.pose.orientation);
		
		// Call runtine here
		odoLaser->estimate_Laser_Odom(&ls);
		
		// See the player and the psm system coordinates!
		// x = y; y = -x;
		odometryData.pose.pose.position.x = ls.y;
		odometryData.pose.pose.position.y = -ls.x;
		odometryData.pose.pose.orientation = tf::createQuaternionMsgFromYaw(ls.theta);   
		
		odometry_in_base(theta);
		pub_1.publish(odometryData);
		}
		incrementa_contador();

		
	}
	
}

void laser2odometry::SubscribeAndPublish::return_to_vero () {		
	nav_msgs::Odometry odom_1 = odometryData;
	geometry_msgs::PointStamped laser_point;
	geometry_msgs::PointStamped vero_point;
	laser_point.header.stamp = odometryData.header.stamp;
	laser_point.header.frame_id = "hokuyo";
	laser_point.point.z = odometryData.pose.pose.position.z;
	laser_point.point.x = odometryData.pose.pose.position.x;
	laser_point.point.y = odometryData.pose.pose.position.y;
	try { 
	   listener.transformPoint("base_link", vero_point.header.stamp, laser_point, "hokuyo", vero_point);
	} catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }


	odom_1.pose.pose.position.x = vero_point.point.x;
	odom_1.pose.pose.position.y = vero_point.point.y;
	odom_1.pose.pose.position.z = vero_point.point.z;

	odometryData = odom_1;
}

void laser2odometry::SubscribeAndPublish::go_to_hokuyo (const nav_msgs::Odometry& odom) {		
	nav_msgs::Odometry odom_1 = odom;
	geometry_msgs::PointStamped laser_point;
	geometry_msgs::PointStamped vero_point;
	vero_point.header.stamp = odom.header.stamp;
	vero_point.header.frame_id = "base_link";
	vero_point.point.z = odom.pose.pose.position.z;
	vero_point.point.x = odom.pose.pose.position.x;
	vero_point.point.y = odom.pose.pose.position.y;
	try { 
	   listener.transformPoint("hokuyo", laser_point.header.stamp, vero_point, "base_link", laser_point);
	} catch (tf::TransformException ex) {
	    ROS_ERROR("%s",ex.what());
	}


	odom_1.pose.pose.position.x = laser_point.point.x;
	odom_1.pose.pose.position.y = laser_point.point.y;
	odom_1.pose.pose.position.z = laser_point.point.z;

	odometryData = odom_1;
}

void laser2odometry::SubscribeAndPublish::hokuyo_relative_to_base (double theta) {
	double matrix[3][3];
	matrix[0][0] = cos(theta);
	matrix[0][1] = -sin(theta);
	matrix[0][2] = 0;

	
	matrix[1][0] = sin(theta);
	matrix[1][1] = cos(theta);
	matrix[1][2] = 0;

	
	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = 1;	
	
	double rot[3];
	rot[0] = 0.135;
	rot[1] = 0;
	rot[2] = 0.25;
	
	double result[3];
	result[0] = rot[0] * matrix[0][0] + rot[1] * matrix[0][1] + rot[2] * matrix[0][2];
	result[1] = rot[0] * matrix[1][0] + rot[1] * matrix[1][1] + rot[2] * matrix[1][2];
	result[2] = rot[0] * matrix[2][0] + rot[1] * matrix[2][1] + rot[2] * matrix[2][2];
	
	odometryData.pose.pose.position.x += result[0];
	odometryData.pose.pose.position.y += result[1];
	odometryData.pose.pose.position.z += result[2];
	
	
}

void laser2odometry::SubscribeAndPublish::odometry_in_base (double theta) {
	double matrix[3][3];
	matrix[0][0] = cos(theta);
	matrix[0][1] = -sin(theta);
	matrix[0][2] = 0;

	
	matrix[1][0] = sin(theta);
	matrix[1][1] = cos(theta);
	matrix[1][2] = 0;

	
	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = 1;	
	
	double rot[3];
	rot[0] = -0.135;
	rot[1] = 0;
	rot[2] = -0.25;
	
	double result[3];
	result[0] = rot[0] * matrix[0][0] + rot[1] * matrix[0][1] + rot[2] * matrix[0][2];
	result[1] = rot[0] * matrix[1][0] + rot[1] * matrix[1][1] + rot[2] * matrix[1][2];
	result[2] = rot[0] * matrix[2][0] + rot[1] * matrix[2][1] + rot[2] * matrix[2][2];
	
	odometryData.pose.pose.position.x += result[0];
	odometryData.pose.pose.position.y += result[1];
	odometryData.pose.pose.position.z += result[2];
	
	
}
