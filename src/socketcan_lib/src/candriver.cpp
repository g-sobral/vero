
#include "candriver.h"
namespace socketcan_lib {

/**
 * \brief reads parameters and confgures socket.
 * @param CAN_Timeout_ms timeout in milisseconds
 * @param InterfaceCAN: the following was copied from socketcan/can-utils/candump.c
 * \par CAN interfaces with optional filter sets can be specified on the commandline in the form:
 * \li <ifname>[,filter]*
 * \par Comma separated filters can be specified for each given CAN interface:
 * \li <can_id>:<can_mask> (matches when <received_can_id> & mask == can_id & mask)
 * \li <can_id>~<can_mask> (matches when <received_can_id> & mask != can_id & mask)
 * \li #<error_mask>       (set error frame filter, see include/linux/can/error.h)
 * \par CAN IDs, masks and data content are given and expected in hexadecimal values. When can_id and can_mask are both 8 digits, they are assumed to be 29 bit EFF. Without any given filter all data frames are received ('0:0' default filter).
 * \par Use interface name 'any' to receive from all CAN interfaces.
 * \par Examples:
 * \li candump -c -c -ta can0,123:7FF,400:700,#000000FF can2,400~7F0 can3 can8
 * \li candump -l any,0~0,#FFFFFFFF    (log only error frames but no(!) data frames)
 * \li candump -l any,0:0,#FFFFFFFF    (log error frames and also all data frames)
 * \li candump vcan2,92345678:DFFFFFFF (match only for extended CAN ID 12345678)
 * \li candump vcan2,123:7FF (matches CAN ID 123 - including EFF and RTR frames)
 * \li candump vcan2,123:C00007FF (matches CAN ID 123 - only SFF and non-RTR frames)
 */
CANDriver::CANDriver(Context prop, bool verbose)
 :verbose_(verbose) {
   // configure can socket
	_prop = prop;
   ifcan_ = prop.interfaceCan;
   int timeout = prop.timeOut;
   devcan_ = prop.devCan;
   int baud = prop.baudRate;

  switch (baud) {
    case 1000:
      baudrate_=CAN1M; break;
    case 500:
      baudrate_=CAN500K; break;
    case 250:
      baudrate_=CAN250K; break;
    case 125:
      baudrate_=CAN125K; break;
    case 100:
      baudrate_=CAN100K; break;
    case 50:
      baudrate_=CAN50K; break;
    case 20:
      baudrate_=CAN20K; break;
    case 10:
      baudrate_=CAN10K; break;
    case 5:
      baudrate_=CAN5K; break;
    default:
	ROS_INFO("CANDriver can not be created, invalid baud rate. Valid values are 5,10,20,50,100,125,250,500, 1000 Kbps. You wrote: %d",baud);
    exit(-1);
  }//switch

   timeval tout;
   tout.tv_sec =  (timeout / 1000);
   tout.tv_usec = (timeout- (tout.tv_sec*1e6) )*1000;

   s_.createSocket(ifcan_,&tout);
   ROS_INFO("CANDriver created: time out %d .%ds if: %s",int(tout.tv_sec), int (tout.tv_usec), ifcan_.c_str());
   network_start();
   setBaudRate(baudrate_);
}//constru

/**
 * \brief sets the baudrate of the can device by writing a parameter in a /dev/ file which is created by the pcan module.
 * @param newbaud this enum type has the possible baudrate values
 * @returns 0 if ok, negative if error
 * \par the baudrate command is "i 0x<divisorvalue> e" There is a corresponding divisor value for each baud rate
 */
int CANDriver::setBaudRate(SocketCAN_Baud newbaud) {
  std::ofstream devfile(devcan_.c_str());
  if (devfile.is_open())
  {
    devfile << "i ";
    switch (newbaud) {
      case CAN1M:
	devfile << CAN_BAUD_1M; break;
      case CAN500K:
	devfile << CAN_BAUD_500K; break;
      case CAN250K:
	devfile << CAN_BAUD_250K; break;
      case CAN125K:
	devfile << CAN_BAUD_125K; break;
      case CAN100K:
	devfile << CAN_BAUD_100K; break;
      case CAN50K:
	devfile << CAN_BAUD_50K; break;
      case CAN20K:
	devfile << CAN_BAUD_20K; break;
      case CAN10K:
	devfile << CAN_BAUD_10K; break;
      case CAN5K:
	devfile << CAN_BAUD_5K; break;
    }//switch
    devfile << " e"<<std::endl;//echo outputs a trailing newline by default (it is really necessary)
    devfile.close();
  } else return -1;
  return 0;
}//setBaudRate

/**
 * \brief setup the network interface by executing ifconfig <can_network_interface> up.
 * \par the network interface is defined by the .cfg option InterfaceCAN. Only the first part of this string is used, cutting on the first ',' to filter out filtering information.
 */
int CANDriver::network_start(){
  std::stringstream ss; // store the complete string to be executed
  ss << "sudo ifconfig " ;
  size_t found=ifcan_.find(',');
  if (found!=std::string::npos) {
    ss << ifcan_.substr(0,found); // if there is a ',' then cut the string
  } else {
    ss << ifcan_; // no filter, it is safe to use the complete string
  }
  ss << " up";
  int ret = system(ss.str().c_str()); // execute ifconfig

  //----------- print message
	ROS_DEBUG("Executed \" %s \" got %d", ss.str().c_str(), ret);

  return ret;
}

/**
 * \brief receives can frames and calls decode for each one. keeps trying to get can frames until it gets an error (or timeout)
 * @param nframes if nframes>0, it will stop when nframes are read. if nframes <= 0, it will read until it timeouts. IT ALWAYS is overwritten with the number of frames actually read.
 * @return the nuimber of can frames read.
 */
int CANDriver::readCAN(unsigned int &nframes) {
  can_frame frame; unsigned int frmct=0; std::string err; int ret;
  bool read_limit = (nframes>0);
  while (0==(ret=s_.rcvCAN(&frame))) {
    decode(frame);
    frmct++;
    if ((read_limit)&&(frmct>=nframes)) break;
  }
  nframes=frmct; // nframes always returns the number of frames read
  //------ print error message
  std::stringstream ss;
  switch (ret) {
    case SOCKCAN_TIMEOUT:
      ss << "Timeout";
      break;
    case SOCKCAN_READ_INCOMPLETE:
      ss << "Read Incomplete";
      break;
    case SOCKCAN_NOT_BOUND:
      ss << "Not Bound";
      break;
    case SOCKCAN_NO_SOCK_ACTIVITY:
      ss << "No Sock Activity";
      break;
    case SOCKCAN_OK:
	// do nothing! we are fine
      break;
    default:
      ss << "CANDriver::read() Unknown error code: "<<ret;
  }//switch
  if ((ret!=SOCKCAN_OK)&&(ret!=SOCKCAN_TIMEOUT))//timeout happens all the time, do not print it
	ROS_INFO ("%s", ss.str().c_str());
  return ret;
}//readCAN

int CANDriver::readData(unsigned int &nframes)
	{ return readCAN(nframes); } 

int CANDriver::writeCanFrame(can_frame &fr) { return s_.wrtCAN(fr); }
void CANDriver::setVerbose(bool verb) {verbose_=verb;}

/**
 * \brief called with each can_frame received. It only prints the can_frame IF verbose is true, ELSE does nothing
 * @uses lib.cpp in libsocketcan_lib to pretty-print the can_frame, (copied from lib.c in socketcan/can-utils/lib.c)
 */
int CANDriver::decode(can_frame frame) {
  if (!verbose_) return 0;
	ros::Time ts = ros::Time::now();
  fprintf(stdout,"%.6f ",float(ts.sec));
  char eol[2]; eol[0]='\n';eol[1]=0;// writte like this to avoid warning. it must also have a 0 after the \n
  fprint_long_canframe(stdout, &frame,eol, 0);
  return 0;
}//decode

} //namespace
