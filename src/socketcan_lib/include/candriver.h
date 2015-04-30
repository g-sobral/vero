
#ifndef HYDRO_CANDRIVER_H
#define HYDRO_CANDRIVER_H

#include "socketcan.h"
#include "caninterface.h"
#include <fstream> //ofstream

#include "ros/ros.h"
#include "ros/time.h"

#include "lib.h"

namespace socketcan_lib
{

/// The possible values for the can baudrate.
enum SocketCAN_Baud { CAN1M,CAN500K,CAN250K,CAN125K,CAN100K,CAN50K,CAN20K,CAN10K,CAN5K};

/// the corresṕonding divisors to be programmed
#define  CAN_BAUD_1M   "0x0014"
#define  CAN_BAUD_500K "0x001C"
#define  CAN_BAUD_250K "0x011C"
#define  CAN_BAUD_125K "0x031C"
#define  CAN_BAUD_100K "0x432F"
#define  CAN_BAUD_50K  "0x472F"
#define  CAN_BAUD_20K  "0x532F"
#define  CAN_BAUD_10K  "0x672F"
#define  CAN_BAUD_5K   "0x7F7F"


/**
 * \brief base class for hydro drivers using CAN. Your specific HydroDriver should inherit this
 * \par it connects to the CAN bus and may print out the received messages
 * \par This class is not specific to the Car Freedom, it should be used to other can stuff
 * @uses SocketCAN it includes a SocketCAN object to handle the communication
 * @see constructor for filter string format.
 * \par IMPORTANT: this class can be used in two ways.
 * \li as a socketcan_lib::caninterface it should be used to read canframes. In this case you should not include this file, include <socketcan_lib::caninterface.h> , like candump does. In this way candump can choose between this simple driver and CANPrint, which prints much more pretty messages specifc to the car. In this way you are using this class, not inheriting it.
 * \li as a CANDriver itself. in this case you should include this file directly, like the hydrodriver hydrocarfreedom does. Actually hydrocarfreedom::Driver inherits this class. In this way you are inheriting this class.
 */
class CANDriver {
    public:
      CANDriver(Context prop, bool verbose=true);
      ~CANDriver (){};
		Context _prop;
      /**
       * \brief called for each can_frame received. Chidrel should reimplement, as this one only prints out the can frame
       */
      virtual int decode(struct can_frame frame);
      virtual int readData(unsigned int &nframes);
	//{ return readCAN(nframes); } ///only calls readCAN. Children can implement similar methods such as readData(hydrointerfaces::CarData &data) or reimplement the decode function to handle specific data. The new function should call readCAN, and then fetch the data from internal attributes (decode will be called by readCAN and write the values)
      virtual int writeCanFrame(can_frame &fr);// { return s_.wrtCAN(fr); }
      void setVerbose(bool verb);// {verbose_=verb;}
      int setBaudRate(SocketCAN_Baud newbaud);
      int network_start();
    protected:
      int readCAN(unsigned int &nframes);
      SocketCAN s_;/// actual socket class
      bool verbose_;/// if set decode will print out the received frames
      std::string devcan_; ///name of the device in /dev that is used to configure the baudrate.
      std::string ifcan_; ///name the can network itnerface
      SocketCAN_Baud baudrate_;/// the baudrate_ utilized in the can bus.
};// class

}//namespace

#endif

//struct can_frame {
//	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
//	__u8    can_dlc; /* data length code: 0 .. 8 */
//	__u8    data[8] __attribute__((aligned(8)));
//};

