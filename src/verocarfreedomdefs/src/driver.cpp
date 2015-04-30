#include <verocarfreedomdefs/driver.h>

using namespace std;
using namespace verocarfreedomdefs;

namespace verocanprint
{

#define prtshort(x) (x>32000?(-(65536-x)):x)

Driver::Driver( const socketcan_lib::Context& context) : CANDriver(context,"") {
}//constructor

Driver::~Driver() {}


/**
 * \brief decodes can frames, reimplements from CanDriver. Checks the ID of each frame and print the frame.
 * @uses calls functionx from canfreedom_defs to extract data from each type of can frame
 */
int Driver::decode(struct can_frame frame) {
  CAN_decode_error canerr= CANFRAME_OK;
  unsigned char counter=0; //unsigned char encerr=0;
  //CANDriver::decode(frame); // print it out, just during tests
  ROS_INFO("%.6f", ros::Time::now().toSec());
  fprint_long_canframe(stdout, &frame,NULL, 0);// print but do not use eol
  for (int i=frame.can_dlc ; i<=8 ; i++)
    cout << "   "; // complete with spaces if the data lenght is small.

  // prints a message for each can_frame type
  switch (frame.can_id) {
    case ID_QUERY_SUPERVISOR:
      cout << "Q_SUPERV";
      break;
    case ID_QUERY_MOTOR_LEFT:
      cout << "Q_MOTR_L";
      break;
    case ID_QUERY_MOTOR_RIGHT:
      cout << "Q_MOTR_R";
      break;
    case ID_QUERY_STEERING_CTRL:
      cout << "Q_ST_CTR";
      break;
    case ID_QUERY_STEERING_ENCODER:
      cout << "Q_ST_ENC";
      break;
    case ID_QUERY_ENCODER_FRONT_LEFT:
      cout << "Q_ENC_FL";
      break;
    case ID_QUERY_ENCODER_FRONT_RIGHT:
      cout << "Q_ENC_FR";
      break;
    case ID_QUERY_ENCODER_REAR_LEFT:
      cout << "Q_ENC_RL";
      break;
    case ID_QUERY_ENCODER_REAR_RIGHT:
      cout << "Q_ENC_RR";
      break;
    case ID_DATA_SUPERVISOR:
      canerr=extractSupervisorFrame(frame,data_);
      cout << "D_SUPERV" << " M ";
      switch (data_.mode) {
	case 1:
	  cout << "REM "; break;
	case 2:
	  cout << "LOC "; break;
	case 3:
	  cout << "AUT "; break;
	default:
	  cout << "ERR ";
      }//switch
      cout << "Bat: "<< data_.battVolt ;
      break;
    case ID_DATA_MOTOR_LEFT:
      canerr=extractDataMotorFrame(frame,data_);
      cout << "D_MOTR_L" << " A " << data_.ampereMotorLeft ;
      break;
    case ID_DATA_MOTOR_RIGHT:
      canerr=extractDataMotorFrame(frame,data_);
      cout << "D_MOTR_R" << " A " << data_.ampereMotorRight ;
      break;
    case ID_DATA_STEERING_CTRL:
      canerr=extractDataSteeringCtrlFrame(frame,data_);
      cout << "D_ST_CTR" << " A " << data_.ampereSteering ;
      cout << (data_.endCourseSteering?" END ":" OK ") ;
      break;
    case ID_DATA_STEERING_ENCODER:
      canerr=extractDataSteeringEncoderFrame(frame,data_);
      cout << "D_ST_ENC" << " s " << data_.steerAngle ;
      break;
    case ID_DATA_ENCODER_FRONT_LEFT:
      canerr=extractDataMotorEncoderFrame(frame,data_);
      cout << "D_ENC_FL" << " e " << data_.speedLeftFront ;
      break;
    case ID_DATA_ENCODER_FRONT_RIGHT:
      canerr=extractDataMotorEncoderFrame(frame,data_);
      cout << "D_ENC_FR" << " e " << data_.speedRightFront ;
      break;
    case ID_DATA_ENCODER_REAR_LEFT:
      canerr=extractDataMotorEncoderFrame(frame,data_);
      cout << "D_ENC_RL" << " e " << data_.speedLeft ;
      break;
    case ID_DATA_ENCODER_REAR_RIGHT:
      canerr=extractDataMotorEncoderFrame(frame,data_);
      cout << "D_ENC_RR" << " e " << data_.speedRight ;
      break;
    case ID_DATA_ENCODER_PULSES_REAR_RIGHT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,counter,data_);
      cout << "D_ECP_RR" << " p " <<  prtshort(data_.speedRight) << " c " << (int)counter;
      break;
    case ID_DATA_ENCODER_PULSES_REAR_LEFT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,counter,data_);
      cout << "D_ECP_RL" << " p " << prtshort(data_.speedLeft) << " c " << (int)counter;
      break;
    case ID_DATA_ENCODER_PULSES_FRONT_RIGHT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,counter,data_);
      cout << "D_ECP_FR" << " p " << prtshort(data_.speedRightFront) << " c " << (int)counter;
      break;
    case ID_DATA_ENCODER_PULSES_FRONT_LEFT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,counter,data_);
      cout << "D_ECP_FL" << " p " << prtshort(data_.speedLeftFront) << " c " << (int)counter;
      break;
    case ID_CMD_SUPERVISOR:
      cout << "C_SUPERV M:";
      switch (frame.data[0]) {
	case 1:
	  cout << " REM "; break;
	case 2:
	  cout << " LOC "; break;
	case 3:
	  cout << " AUT "; break;
	default:
	  cout << " ERR "; break;
      }//switch
      cout << (frame.data[1]?" L ":" l ");
      cout << (frame.data[2]?" B ":" b ");
      break;
    case ID_CFG_MOTOR_LEFT:
      cout << "C_CFGM_L a:" << frame.data[0];
      break;
    case ID_CFG_MOTOR_RIGHT:
      cout << "C_CFGM_R a:" << frame.data[0];
      break;
    case ID_CMD_MOTOR_LEFT:
      cout << "C_MOTR_L :" << frame.data[0] + frame.data[1]*255;
      cout << (frame.data[2]?" B ":" b ");
      break;
    case ID_CMD_MOTOR_RIGHT:
      cout << "C_MOTR_R :" << frame.data[0] + frame.data[1]*255;
      cout << (frame.data[2]?" B ":" b ");
      break;
    case ID_CMD_MOTOR_JOYSTICK: // not generated by the embsys, it comes from the supervisor (joystick)
      cout << "C_MOTR_J :" << frame.data[0] + frame.data[1]*255;
      break;
    case ID_CMD_MOTOR_RPH:
      switch (frame.can_dlc) {//this one may have two different sizes.
	case 5:
	  cout << "C_MOTRPH R:" << (frame.data[1] + frame.data[2]*255)/60 << " L: " << (frame.data[3] + frame.data[4]*255)/60;
	  break;
	case 3:
	  cout << "C_MOTRPH FW:" << (frame.data[1] + frame.data[2]*255)/60;
	  break;
	default:
	  cout << "C_MOTRPH ERR " << frame.can_dlc ;
      }//switch
      break;
    case ID_CMD_STEERING:
      cout << "C_STEERI :" << frame.data[0] + frame.data[1]*255;
      break;
    case ID_CFG_ENCODER_PERIOD_REAR_RIGHT:
      cout << "C_ECP_RR :" << frame.data[0] ;
      break;
    case ID_CFG_ENCODER_PERIOD_REAR_LEFT:
      cout << "C_ECP_RL :" << frame.data[0] ;
      break;
    case ID_CFG_ENCODER_PERIOD_FRONT_RIGHT:
      cout << "C_ECP_FR :" << frame.data[0] ;
      break;
    case ID_CFG_ENCODER_PERIOD_FRONT_LEFT:
      cout << "C_ECP_FL :" << frame.data[0] ;
      break;
  }//switch
  cout << endl;// one endl to end them all
  return 0;
}//decode



} // namespace
/*
extern "C" {
    //hydrocanprint::Factory *createDriverFactory()
    verohydrointerfaces::CANInterfaceFactory *createDriverFactory()
    { return new hydrocanprint::Factory; }
}

*/

