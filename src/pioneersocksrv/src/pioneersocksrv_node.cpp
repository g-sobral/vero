

//
// daytime_server.cpp
// ~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <tf/transform_datatypes.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

using boost::asio::ip::tcp;


/**
 * \brief this singleton handles global callbacks and stores data to send back to MATLAB
 * \par yep, this is the nice way to handle global callbacks and data. 
 */
class DataSingleton {
private:
  static DataSingleton* instance; /// There can be only one. 
  
protected:
  /// keep my constructor protected:, I am a Singleton !!!
  DataSingleton(){
    gotodo = false; 
    gotpantilt = false;
  }
public:
  /// @see Singleton pattern. This guarantees that there can be only one.
  static DataSingleton* uniqueInstance() {
    if (NULL==instance) instance = new DataSingleton();
    return instance;
  }//uniqueInstance 

  /// callback calls this to store the last odo
  void setOdometry(const nav_msgs::Odometry odo) { odo_=odo; gotodo=true; ROS_INFO_THROTTLE(5,"set odo_ [%f]",odo_.pose.pose.position.x);}
  /// callback calls this to store the last pantilt velocity state
  void setTwist(const geometry_msgs::Twist pantilt) { pantilt_=pantilt; gotpantilt = true; }
  void setPose(const geometry_msgs::Pose pos) { ptlPose_=pos; }
  
  
  /**
   * \brief  mounts the message to send back to matlab. 
   * \par The singleton just stores all data, and this function uses the data to generate a message
   */
  std::string getMsg() {
    std::string m;
    if ((!gotodo)&&(!gotpantilt)) {
      ROS_WARN("getMsg called but it has no data to send");
    }
    
    /// ros msg all use quaternions
    tf::Quaternion q(odo_.pose.pose.orientation.x,odo_.pose.pose.orientation.y,odo_.pose.pose.orientation.z,odo_.pose.pose.orientation.w);
    tf::Quaternion qptl(ptlPose_.orientation.x,ptlPose_.orientation.y,ptlPose_.orientation.z,ptlPose_.orientation.w); 
    double roll,pitch,yaw; // so we use tf:: classes to convert to RPY angles
    tf::Matrix3x3(qptl).getRPY(roll, pitch, yaw);
    
    // HERE THE CORE. ASSEMBLE MESSAGE FROM DATA
    char buf[100]; buf[99]=0;
    sprintf(buf," %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",
	odo_.pose.pose.position.x,odo_.pose.pose.position.y,odo_.pose.pose.position.z,
	tf::getYaw(q),pitch,yaw,
	pantilt_.linear.x,pantilt_.linear.y,pantilt_.linear.z,
	pantilt_.angular.x,pantilt_.angular.y,pantilt_.angular.z
	  );
    msgsend_ = buf;
    return msgsend_;
  }//getMsg
  
private:
  std::string msgsend_; 
  nav_msgs::Odometry odo_;
  geometry_msgs::Twist pantilt_;
  geometry_msgs::Pose ptlPose_;
  bool gotodo;
  bool gotpantilt;
}; //class
  
DataSingleton* DataSingleton::instance=NULL;  

void odoCallback(const nav_msgs::Odometry odo){
   ROS_INFO_THROTTLE(5,"odoCallback [%f]",odo.pose.pose.position.x);
  DataSingleton* ds = DataSingleton::uniqueInstance();
  ds->setOdometry(odo);
} // odoCallback


void pantiltCallback(const geometry_msgs::Twist tw){
  DataSingleton* ds = DataSingleton::uniqueInstance();
  ds->setTwist(tw);
} // pantiltCallback

void pantiltPoseCallback(const geometry_msgs::Pose pos){
  DataSingleton* ds = DataSingleton::uniqueInstance();
  ds->setPose(pos);
} // pantiltCallback


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "pioneersocksrv");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Publisher ptl_pub = n.advertise<geometry_msgs::Twist>("/PTU_cmd_vel", 1000);
  ros::Subscriber subodo = n.subscribe("/pose", 1000, odoCallback);
  ros::Subscriber subptl = n.subscribe("/PTU46_velocity", 1000, pantiltCallback);
  ros::Subscriber subptlpose = n.subscribe("/PTU46_pose", 1000, pantiltPoseCallback);
  DataSingleton* ds = DataSingleton::uniqueInstance() ;
  try
  {
    boost::asio::io_service io_service; /// look for boost tcp stuff
    double a,b;
    tcp::endpoint endpoint(tcp::v4(), 1111);
    tcp::acceptor acceptor(io_service, endpoint);

    while (ros::ok())
    {
      tcp::iostream stream;
      boost::system::error_code ec;
      acceptor.accept(*stream.rdbuf(), ec);
      char rec[1000];
      
      //std::stringstream send;
      
      if (!stream) {
      	ROS_ERROR("Error accepting socket: %s \n", stream.error().message().c_str() );
	continue;
      } 
      
      try {
	while (ros::ok) {
	  ros::spinOnce();/// process callbacks, fill data 
	  if (!ec) {
	    //----------- receiving message from matlab
	    stream.getline(rec,1000);
	    ROS_INFO("got %s",rec);
	    geometry_msgs::Twist msg_cmdvel;///ros msgs to send
	    geometry_msgs::Twist msg_cmdptl;///ros msgs to send
	    double aux;
	    //--------- core: PARSE RECEIVED MESSAGE
	    int nscan = sscanf(rec,"%lf %lf %lf %lf %lf %lf "
			      ,&msg_cmdvel.linear.x,&msg_cmdvel.angular.z,&aux
			      ,&msg_cmdptl.angular.x,&msg_cmdptl.angular.y,&msg_cmdptl.angular.z);
	    if (nscan == 6)  { 
	      msg_cmdvel.linear.y = msg_cmdvel.linear.x; // not sure which axis to use, so both are the same. 
	      vel_pub.publish(msg_cmdvel); /// actual publishing
	      ptl_pub.publish(msg_cmdptl);
	    }
	    //---------- sending message to matlab ------------------
	    ROS_INFO("sending %s",ds->getMsg().c_str());
	    stream << ds->getMsg();
	    if (!stream) {
	      ROS_ERROR("Error: %s \n", stream.error().message().c_str() );
	      stream.close();
	      throw std::ios_base::failure(stream.error().message());
	    }
	  }//if (!ec)
	}//while
      } catch  (std::exception& e)
      {
	std::cerr <<"Internal catch " << e.what() << std::endl;
      } // try
    }//while
  }
  catch (std::exception& e)
  {
    std::cerr << "Ext catch " << e.what() << std::endl;
  }

  return 0;
}
