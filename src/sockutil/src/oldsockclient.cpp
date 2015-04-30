
#include "sockutil/sockclient.h"

using namespace sockutil;

SockClient::SockClient(std::string host,int port) : host_(host), port_(port) {}
SockClient::~SockClient(){}

SocketFactory* SocketFactory::instance=NULL;/// just initialization 


int SockClient::connect() {
  std::string sport = boost::lexical_cast<std::string>(port_);
  s.connect(host_,sport);
  // there is a !operator to know if the connection was stabilished
  if (!s) { /// ERROR!!!
    return -1;
  }
  return 0;
}// connect 


int SockClient::close() {
  s.close();
  return 0;
}
int SockClient::send(std::string &msg){
    // there is a !operator to know if the connection was stabilished
  if (!s) {
    ROS_WARN_THROTTLE(2,"Socket is brocken, trying to reconect at %s:%d!",host_.c_str(),port_);
    connect();
  }
  s << msg;
  //s.flush();
  return 0;
}//send
 
int SockClient::rec(std::string &msg, int &lastread){
  std::streambuf* pbuf = s.rdbuf();// this is the underlying buffer
  // this DOES NOT WORK. IN_AVAIL IS ALWAYS ZERO TODO
  std::streamsize size = pbuf->in_avail(); // ask how many bytes there are in the buffer to be read
  while (size > 0 ) {
    // read many times until the buffer is empty. 
    rec_blk(msg); // overwrite old ones and return only the last message.
    size = pbuf->in_avail();
    lastread = size;
  }//while
  return 0;
}// rawrec
  
int SockClient::rec_blk(std::string &msg){
  std::getline(s, msg);
  return 0;
} // rawrec_blk  


int StaubliClient::send(std::vector<double> &v_c) {
    if (v_c.size() != 6) {// check if the pose has 6 elements
      ROS_ERROR_THROTTLE(2,"StaubliClient:: v_c must have 6 elements, but it has %lu",v_c.size());
      return -1;
    }
    
    sprintf(buf,"%.3f %.3f %.3f %.3f %.3f %.3f \n\0",v_c[0],v_c[1],v_c[2],v_c[3],v_c[4],v_c[5]);
    msg_ = buf;
    return SockClient::send(msg_);
  }// send


int StaubliClient::send(boost::array<double,6> &v_c) {
    sprintf(buf,"%.3f %.3f %.3f %.3f %.3f %.3f \n\0",v_c[0],v_c[1],v_c[2],v_c[3],v_c[4],v_c[5]);
    msg_ = buf;
    return SockClient::send(msg_);
  }// send boost::array
  
  
  int StaubliClient::rec(std::vector<double> &vel, std::vector<double> &here) {
    int sizeread;
    if (vel.size() != 6) vel.resize(6);
    if (here.size() != 6) here.resize(6);
    SockClient::rec(msg_, sizeread);
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
    // this is bad. need a separate thread
    std::getline(s, msg_);
    
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

  