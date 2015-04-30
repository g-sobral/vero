
#ifndef SOCKCLIENT_H
#define SOCKCLIENT_H

#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <boost/lexical_cast.hpp> // convertion int -> string
#include <boost/thread/thread.hpp>
#include "rawclient.cpp"

using boost::asio::ip::tcp;

namespace sockutil { 
  
/**
 * \brief base class for simple socket communication
 * \par uses boost iostream socket
 * \li http://www.boost.org/doc/libs/1_46_0/doc/html/boost_asio/example/iostreams/daytime_client.cpp
 * \par another reference about how to check for success and expiration
 * \li http://www.boost.org/doc/libs/1_47_0/doc/html/boost_asio/overview/networking/iostreams.html
 */
class SockClient {
public:
  
  
//virtual int send(std::vector<double> &v_c)=0;
//virtual int rec(std::vector<double> &vel, std::vector<double> &here)=0;

  SockClient(std::string host,int port);
  ~SockClient();
  
  int connect();
  
  //std::string errmsg() { return s.error().message();  }
  
  int close() ;
virtual int send(std::string &msg);
  
virtual int rec(std::string &msg);

  
  std::string getHost() { return host_; } 
  int getPort() {return port_; }

protected:
  void run();
  char buf[250];
  std::string host_;
  int port_;
  bool stop;
  bool new_write;
  
  client *rawclientptr;
  boost::thread thread_;
  
}; //class

/**
 * \brief uses SockClient to comunicate with the Staubli robot
 * \par sends velocities (6-DOF), and receives current velocities and pose (both 6-DOC). In total, it sends 6 floats and receives 12
 */
class StaubliClient : public SockClient {
public:
  StaubliClient(std::string host,int port) : SockClient(host,port) { msg_.resize(200); }
  int send(std::vector<double> &v_c) ;
  int send(boost::array<double,6> &v_c) ;

  int rec(std::vector<double> &vel, std::vector<double> &here) ;
  int rec(boost::array<double,6> &vel, boost::array<double,6> &here) ;

private:
  std::string msg_;
  
}; // class


/**
 * \brief factory method implemented as a singleton. Ask the singleton to create a socket client for you. 
 */
class SocketFactory {
private:
  static SocketFactory* instance;
protected:
  SocketFactory(){}
public:
  
  static SocketFactory* uniqueInstance() {
    if (NULL==instance) instance = new SocketFactory();
    return instance;
  }//uniqueInstance
  /// if you have a new subclass, define here when it is created.
  SockClient* newSockClient(std::string &robot,std::string &host, int port) {
    if (robot == "staubli" ) {
	return new StaubliClient(host,port);
    }
    ROS_ERROR("SocketFactory:: unknown robot name: %s", robot.c_str());
    return NULL;
  } 
  
  
}; //class



}//namespace
#endif