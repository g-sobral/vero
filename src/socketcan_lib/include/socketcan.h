
#ifndef HYDRO_SOCKETCAN_H
#define HYDRO_SOCKETCAN_H

#include <string>
#include <string.h>//memset
#include <iostream>
#include <sstream>
#include <stdio.h> // sscanf
#include <sys/ioctl.h>//ioclt
#include <sys/socket.h>
#include <net/if.h>//ifreq

#include <linux/can.h>
#include <linux/can/raw.h>

#include "ros/ros.h"

// PF_CAN is in sys/socket.h but some ifdefs do not allow us to see them
// it looks like linux does not like different protocols outside of the kernel
#define PF_CAN 29
#define AF_CAN PF_CAN

namespace socketcan_lib
{

typedef struct Context_ {
	std::string interfaceCan;
	int timeOut;
	std::string devCan;
	int baudRate;
} Context;

void IniciaContext (Context &context);

// ----  error messages
#define SOCKCAN_OK 0
#define SOCKCAN_TIMEOUT -1
#define SOCKCAN_NO_SOCK_ACTIVITY -2
#define SOCKCAN_READ_INCOMPLETE -3
#define SOCKCAN_NOT_BOUND -4

/**
 * \brief encapsulates a socket to receive and send data with the PF_CAN protocol family
 * \par due to the different protocol family, you can not use usual socket libs such as Qt. So this implements a socket layer.
 * \par This class only knows about can_frames and not orca or hydro types. The user class should decode the can_frames
 */
  class SocketCAN {
    public:
      SocketCAN();
      void createSocket(std::string ifid,timeval *timeout,int rcvbuf_size=5000);

      int rcvCAN(struct can_frame *frame) ;
      int wrtCAN(struct can_frame &frame) ;

    protected:
      void internal_createSocket();
      std::string ifid_;/// the name of the can network interface (e.g., can0)
      int sock_family_;/// the socket family by default (PF_CAN)
      int sock_type_;/// the socket type, by default (SOCK_CAN)
      int sock_protocol_; /// the protocol - e.g. CAN_RAW
      bool isBound_;///set after successfull bind
      int s; ///the socket descriptor itself
      struct ifreq ifr;///variable needed for the socket
      struct sockaddr_can addr;///variable needed for the socket
      timeval timeout_;///timeout for select messages
      bool have_timeout;///do we have timeout or block indefinitely?
      int rcvbuf_size_; /// the size of the RX buffer in the socket
  };// class

}//namespace


#endif

//struct can_frame {
//	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
//	__u8    can_dlc; /* data length code: 0 .. 8 */
//	__u8    data[8] __attribute__((aligned(8)));
//};

