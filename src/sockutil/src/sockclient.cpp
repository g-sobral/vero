
#include "sockutil/sockclient.h"


using namespace sockutil;

SockClient::SockClient(std::string host,int port) : host_(host), port_(port), thread_(boost::bind(&SockClient::run, this)), stop(false), new_write(false)  {
  rawclientptr=NULL;
}
SockClient::~SockClient(){}

SocketFactory* SocketFactory::instance=NULL;/// just initialization 


int SockClient::connect() {
  
  
  
  
}// connect 

/**
 * \brief thread which handles socket communication
 * \par see  http://www.boost.org/doc/libs/1_47_0/doc/html/boost_asio/example/timeouts/async_tcp_client.cpp
 * \par this initializes the rawsocket class and runs io_service to process requests to send/receive data.
 * 
 */
void SockClient::run() {
  try
  { 
    boost::asio::io_service io_service;
    client rawclient(io_service); /// this will be always alive. 
    std::string strport = boost::lexical_cast<std::string>(port_);// convert int to string
    tcp::resolver r(io_service);
    rawclient.start(r.resolve(tcp::resolver::query(host_.c_str(), strport)));
    rawclientptr = &rawclient;/// the other functions need a pointer 
    while (!stop) {
      io_service.run(); // run ultill there is nothing more to do in queue
      usleep(1000*4); // 4 ms, half period, but it is not always, only if todo queue is empty
      if (rawclient.isStopped()) {
	// try to restart it
	ROS_WARN_THROTTLE(2,"SOCKET THREAD IS RESTARTING SOCKET!");
	rawclient.start(r.resolve(tcp::resolver::query(host_.c_str(), strport)));
      }
    }
    ROS_ERROR("SOCKET THREAD HAS TERMINATED!");
  }
  catch (std::exception& e)
  {
    ROS_ERROR("SOCKET THREAD HAS TERMINATED with Exception: %s" , e.what());
  }
    
}//run

/// wrapper to rawclient
int SockClient::close() {
  rawclientptr->stop();
  return 0;
}
/// wrapper to rawclient
int SockClient::send(std::string &msg){
  rawclientptr->send(msg);
  return 0;
}//send
/// wrapper to rawclient
int SockClient::rec(std::string &msg){
  rawclientptr->rec(msg);
  return 0;
}// rawrec
  


////////////////////////////////////////////////////////////////////////
///////////////////////// staubli ///////////////////////////////////

int StaubliClient::send(std::vector<double> &v_c) {
    if (v_c.size() != 6) {// check if the pose has 6 elements
      ROS_ERROR_THROTTLE(2,"StaubliClient:: v_c must have 6 elements, but it has %u",v_c.size());
      return -1;
    }
    
    sprintf(buf,"%.3f %.3f %.3f %.3f %.3f %.3f \n",v_c[0],v_c[1],v_c[2],v_c[3],v_c[4],v_c[5]);
    msg_ = buf;
    return SockClient::send(msg_);
  }// send


int StaubliClient::send(boost::array<double,6> &v_c) {
    sprintf(buf,"%.3f %.3f %.3f %.3f %.3f %.3f \n",v_c[0],v_c[1],v_c[2],v_c[3],v_c[4],v_c[5]);
    msg_ = buf;
    //std::cerr << "\nStaubliClient::send " << msg_; 
    return SockClient::send(msg_);
  }// send boost::array
  
  
  int StaubliClient::rec(std::vector<double> &vel, std::vector<double> &here) {
    int sizeread;
    if (vel.size() != 6) vel.resize(6);
    if (here.size() != 6) here.resize(6);
    SockClient::rec(msg_);
    int fieldsread = sscanf(msg_.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"
			     ,&vel[0],&vel[1],&vel[2],&vel[3],&vel[4],&vel[5]
			     ,&here[0],&here[1],&here[2],&here[3],&here[4],&here[5]
 			    );
    if (fieldsread != 12) {
	ROS_ERROR_THROTTLE(2,"the robot should have sent 12 floats, but it has only sent %d fields in %d bytes",fieldsread, sizeread);
	return -1;
    }
    return 0;
  }//rec


  int StaubliClient::rec(boost::array<double,6> &vel, boost::array<double,6> &here) {
    SockClient::rec(msg_);
    //s.flush();
    int sizeread=0;
    //SockClient::rec(msg_, sizeread);
    int fieldsread = sscanf(msg_.c_str(),"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"
			     ,&vel[0],&vel[1],&vel[2],&vel[3],&vel[4],&vel[5]
			     ,&here[0],&here[1],&here[2],&here[3],&here[4],&here[5]
 			    );
    if (fieldsread != 12) {
	ROS_ERROR_THROTTLE(2,"the robot should have sent 12 floats, but it has only sent %d fields in %d bytes",fieldsread, sizeread);
	return -1;
    }
    return 0;
  }//rec

  