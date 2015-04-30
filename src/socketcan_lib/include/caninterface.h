
#ifndef socketcan_lib_CANDRIVER_H
#define socketcan_lib_CANDRIVER_H

#include "socketcan.h"

namespace socketcan_lib
{

/**
 * \brief base class for hydro drivers using CAN. Your specific HydroDriver should inherit this
 * \par it connects to the CAN bus and print out the received messages
 * @uses SocketCAN it includes a SocketCAN object to handle the communication
 * @see constructor for filter string format.
 */
  class CANInterface {
    public:


      /**
       * \brief called for each can_frame received. Chidrel should reimplement, as this one only prints out the can frame
       */
      virtual int decode(struct can_frame frame)=0;
      virtual int readData(unsigned int &nframes)=0;
      virtual int writeCanFrame(can_frame &fr)=0;
      virtual void setVerbose(bool verb)=0;
};// class

            /*! @} */
} // namespace

#endif

//struct can_frame {
//	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
//	__u8    can_dlc; /* data length code: 0 .. 8 */
//	__u8    data[8] __attribute__((aligned(8)));
//};

