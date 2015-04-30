#include "socketcan.h"

namespace socketcan_lib{

/**
 * \brief just sets internal values.
 */
SocketCAN::SocketCAN()
{
  sock_family_=PF_CAN;
  sock_type_=SOCK_RAW;
  sock_protocol_=CAN_RAW;
  isBound_=false;
}//constructor

void IniciaContext (Context &context){
	ros::NodeHandle nh("~");
	nh.param<std::string>("CarFreedom_Config_CarHeartbeat_InterfaceCAN", context.interfaceCan, "can0");
nh.param<int>("CarFreedom_Config_CarHeartbeat_CAN_Timeout_ms", context.timeOut, 1);
nh.param<std::string>("CarFreedom_Config_CarHeartbeat_CAN_Dev_Can", context.devCan, "/dev/pcanusb0");
nh.param<int>("CarFreedom_Config_CarHeartbeat_CAN_Baud_Rate", context.baudRate, 125);
}

/**
 *\brief stores and checks the configuration parameters and then calls internal_createSocket()
 *@param ifid string with the network interface name plus filter definitions
 *@param timeout timeout to be used in can operations. If it is NULL, there will be no timeout, and the operations may block indefinitely. You can set the timeout to zero to have non-blocking calls.
 */
void SocketCAN::createSocket(std::string ifid,timeval *timeout,int rcvbuf_size)
  {
  rcvbuf_size_=rcvbuf_size;
  ifid_=ifid;
  if (timeout) {//we have a timeout, copy it
    timeout_.tv_sec=timeout->tv_sec;timeout_.tv_usec=timeout->tv_usec;
    have_timeout=true;
  } else { // no timeout, it is null
    have_timeout=false;
  }

  //there is a maximum size for the ifdsize
  if (ifid_.size() >= IFNAMSIZ)
  {
		ROS_DEBUG ("socketcan_lib::SocketCan::SocketCan name of CAN network device is too long.");
  }
  internal_createSocket();// the job will actually be done here.
  //std::cout << "ToutCCC: " << timeout_.tv_sec << "." << timeout_.tv_usec << std::endl;
}//createSocket

/**
 *\brief the socket creation magic, plus parsing of the filter definition string.
 *@see copied from the example candump.c from socketcan/can_utils
 *\par yes, socket libs are messy code, but I went into verbose comment mode
 */
void SocketCAN::internal_createSocket(){
  can_err_mask_t err_mask;
  char buf[500];// buffer to store substrings

  s = socket(sock_family_, sock_type_,sock_protocol_);//socket id is created
  if (s < 0) {
	ROS_DEBUG ("socketcan_lib::SocketCan::createSocket Could not create Socket");
  }

  // see if there is comma in the interface name (indicating filters) and count the commas
  size_t comma=0; unsigned int num_filters=0;
  while ((comma=ifid_.find(',',comma+1))!=std::string::npos) {
    num_filters++;
  }
  struct can_filter rfilter[num_filters];//struct to hold filter information. now we know how many filters we have

  comma=ifid_.find(',',0);// gets the position of the first comma

  //--  copy the interface name into ifr.name
  //std::cout << "ifid "<<ifid_<<" comma:" <<comma<<" nfilters "<<num_filters<<std::endl;
  memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));//zero the space for the interface name
  if (comma!=std::string::npos) {//we have filters, so we copy up to the 1st comma
    ifid_.copy(buf,comma,0); buf[comma]=0;//guarantee a zero terminated string
    strncpy(ifr.ifr_name, buf, comma);//copy ifname into the right struct
  } else {//there is no filters. copy the whole ifid_ string
    ifid_.copy(buf,ifid_.size(),0); buf[ifid_.size()]=0;//guarantee a zero terminated string
    strncpy(ifr.ifr_name, buf,ifid_.size());//copy ifname into the right struct
  }

  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		ROS_DEBUG("socketcan_lib::SocketCan::createSocket ioclt error");
  }
  //std::cout << "buf "<<buf<< " ifrn "<< ifr.ifr_name<<std::endl;;
  addr.can_ifindex = ifr.ifr_ifindex;

  int filtersize=0; //store the size of each filter substring
  for (unsigned int filtercount=0; filtercount < num_filters; filtercount++) {//for each filter
    //get the positon of the next comma
    size_t nextcomma = ifid_.find(',',comma+1);
    if (std::string::npos==nextcomma) {//this is the last filter
      nextcomma=ifid_.size();// nextcoma points right after the end
    } //else we still have another filter
    filtersize=nextcomma-comma-1;//calculate the size of the filter substring
    ifid_.copy(buf,filtersize,comma+1);//copy the filter substring to the buf
    buf[filtersize]=0;//guarantee a zero terminated string
    //std::cout << "f "<<filtercount<<"buf "<<buf<< " ifrn "<< ifr.ifr_name<<std::endl;;
    //---- parse the filter substring.
    if (sscanf(buf, "%lx:%lx",
      (long unsigned int *)
      &rfilter[filtercount].can_id,
		(long unsigned int *)
		&rfilter[filtercount].can_mask) == 2) {
      rfilter[filtercount].can_mask &= ~CAN_ERR_FLAG;
    } else if (sscanf(buf, "%lx~%lx",
      (long unsigned int *)
      &rfilter[filtercount].can_id,
		       (long unsigned int *)
		       &rfilter[filtercount].can_mask) == 2) {
      rfilter[filtercount].can_id |= CAN_INV_FILTER;
    rfilter[filtercount].can_mask &= ~CAN_ERR_FLAG;
    } else if (sscanf(buf, "#%lx",
      (long unsigned int *)&err_mask) != 1) {
      std::stringstream ss;
    ss << "socketcan_lib::SocketCan::createSocket Error in filter option parsing:" << buf;
		ROS_DEBUG ("%s", ss.str().c_str());
    }//if sscanf
    //std::cout << "f "<<filtercount<<"buf "<<buf<< " ifrn "<< ifr.ifr_name<<std::endl;;
  }//for

  //------------------------------------
  // -------- add filter to get error messages from the can bus
  if (err_mask) {
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
		&err_mask, sizeof(err_mask));
  }
  //add filter to filter data messages
  if (num_filters) {
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER,
		&rfilter, num_filters * sizeof(struct can_filter));
  }
  //-------------------------------------
  //----------- set buffers in reception
  int curr_rcvbuf_size;
  socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

  if (setsockopt(s, SOL_SOCKET, SO_RCVBUF,
    &rcvbuf_size_, sizeof(rcvbuf_size_)) < 0) {
    std::stringstream ss;
    ss << "socketcan_lib::SocketCan::createSocket setsockopt SO_RCVBUF" ;
    ROS_DEBUG ("%s", ss.str().c_str());
  }

  if (getsockopt(s, SOL_SOCKET, SO_RCVBUF,
    &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0) {
    std::stringstream ss;
    ss << "socketcan_lib::SocketCan::createSocket getsockopt SO_RCVBUF" ;
    ROS_DEBUG ("%s", ss.str().c_str());
  }

  /* Only print a warning the first time we detect the adjustment */
  /* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
  if (curr_rcvbuf_size < rcvbuf_size_*2) {
    std::stringstream ss;
    ss << "socketcan_lib::SocketCan::createSocket The socket receive buffer size was "
    "adjusted due to /proc/sys/net/core/rmem_max." ;
    ROS_DEBUG ("%s", ss.str().c_str());
  }

  // the final bind of the socket.
  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    ROS_DEBUG ("socketcan_lib::SocketCan::createSocket bind error");
  }
  isBound_=true;// if we are here, it should be ok. we are bound!
}//createSocket

/**
 *\brief receive a can frame from the can bus.
 \par it uses select and reads data from the socket. it is only ok if the data read has the correct size of the can frame.
 *@uses the timeout attribute is used in the reading operations
 @return 0 if a can frame was read, non-zero otherwise (timeout, error, wrong data size, uninitialized object)
 * \par Error codes in socketcan.h:
 * \li TIMEOUT -1
 * \li NO_SOCK_ACTIVITY -2
 * \li READ_INCOMPLETE -3
 * \li NOT_BOUND -4
 */
int SocketCAN::rcvCAN(struct can_frame *frame) {
  if (!isBound_)  { return SOCKCAN_NOT_BOUND; }// there is not a socket! lets move out of here.

  fd_set rdfs;//descriptor set for select

  FD_ZERO(&rdfs);
  FD_SET(s, &rdfs);

  int retsel=0;
  if (have_timeout) {
    timeval auxtout = timeout_;// select destroys its timeval parameter, so we need to copy it
    if ((retsel = select(s+1, &rdfs, NULL, NULL, &auxtout)) < 0) {
      ROS_DEBUG ("socketcan_lib::SocketCan::rcvSocket socket error");
    }
  } else {
    if ((retsel = select(s+1, &rdfs, NULL, NULL, NULL)) < 0) {
      ROS_DEBUG ("socketcan_lib::SocketCan::rcvSocket socket error (no timeout)");
    }
  }
  if (0==retsel) {//timeout
    return SOCKCAN_TIMEOUT;
  }
  if (FD_ISSET(s, &rdfs)) {// check if select has actally detected activity in our socket

    socklen_t len = sizeof(addr);

    unsigned int nbytes = recvfrom(s, frame, sizeof(struct can_frame), 0,(struct sockaddr*)&addr, &len);
    if (nbytes < 0) {
      ROS_DEBUG ("socketcan_lib::SocketCan::rcvCAN READ error");
    }
    if (nbytes < sizeof(struct can_frame)) {//check if the data size is correct
     return SOCKCAN_READ_INCOMPLETE;
    }
    return 0;

  } else {
    return SOCKCAN_NO_SOCK_ACTIVITY;
  }
}//rcvCAN

/**
 * \brief simple writing in the socket.
 * @return 0 if writing was ok, non-zero if error
 */
int SocketCAN::wrtCAN(struct can_frame &frame) {
  if (!isBound_) return 1;
  unsigned int nbytes = write(s, &frame, sizeof(struct can_frame));
  if (nbytes < 0) {
    //if (errno != ENOBUFS) {
      ROS_DEBUG("socketcan_lib::SocketCan::wrtCAN write error");
      return 1;
    /*}
    if (!ignore_enobufs) {

      return 1;
    }
    enobufs_count++;
    */
  } else if (nbytes < sizeof(struct can_frame)) {
    //fprintf(stderr, "write: incomplete CAN frame\n");
    return 1;
  }
  return 0;
}//wrtCAN
}//namespace
