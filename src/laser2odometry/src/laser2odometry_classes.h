#ifndef Subscribe_And_Publish
#define Subscribe_And_Publish

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "ros/console.h"                                  
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/LinearMath/Transform.h"
#include "telemetricOdometry.h"

namespace laser2odometry {
class SubscribeAndPublish 	{
public:
	static const int DIV_FREQ_HOKUYO = 5;
	
	SubscribeAndPublish();
	
	/* Salva a última odometria */
	void callback_odom (const nav_msgs::Odometry& odom);
	/* Salva os dados do laser */
	void callback_laser(const sensor_msgs::LaserScan& laserData);
	/* Retorna para o eixo de coordenadas do veículo */
	void return_to_vero ();
	void go_to_hokuyo (const nav_msgs::Odometry& odom);
	void hokuyo_relative_to_base (double theta);
	void odometry_in_base (double theta);
	laserodometry::Scan ls;
	void incrementa_contador ();
	
private:
	
	int contador;	
	ros::NodeHandle n_; 
	ros::Publisher pub_1;
	ros::Subscriber sub_1;
	ros::Subscriber sub_2;
	laserodometry::TelemetricOdometry *odoLaser;
	nav_msgs::Odometry odometryData;
	tf::TransformListener listener;
	bool has_odom;

	
	
	

}; //End of class SubscribeAndPublish
}

#endif
