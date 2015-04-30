
#include "driver.h"

using namespace std;
using namespace verocarfreedomdefs;

namespace verocarfreedom
{
/*
void IniciaContext (Context &context){
	context.CAN_Delay_Between_Queries_us = 0;
	context.Encoder_Pulses_Period_ms = 20;
	context.MaxSteerAngleLeft = CARFREEDOM_MAX_STEERING_ANGLE_DEG;
	context.MaxSteerAngleRight = CARFREEDOM_MAX_STEERING_ANGLE_DEG;
	//context.SteeringAng2CmdEnd1stLine_Adeg_Cmd = Ang2Cmd_End1stLine_;
	//context.SteeringAng2CmdStart2ndLine_Adeg_Cmd = Ang2Cmd_Start2ndLine_;
	context.SteeringAng2Cmd_ZeroCmd = 512;
	context.MaxForwardSpeed = CARFREEDOM_MAX_SPEED_FORWARD_M_S;
	context.MaxReverseSpeed = CARFREEDOM_MAX_SPEED_REVERSE_M_S;
	context.JoystickMaxForward = CARFREEDOM_MAX_JOYSTICK_FORWARD;
	context.JoystickMaxReverse = CARFREEDOM_MAX_JOYSTICK_REVERSE;
	context.JoystickMaxLeft = CARFREEDOM_MAX_JOYSTICK_LEFT;
	context.JoystickMaxRight = CARFREEDOM_MAX_JOYSTICK_RIGHT;
	context.WheelRadius_m = CARFREEDOM_WHEEL_RADIUS_M;
	context.InterAxesDistance_m = CARFREEDOM_INTER_AXES_DISTANCE_M;
	context.AxisLength_m = CARFREEDOM_AXIS_LENGTH_M;
	context.Wheels_V_Uncertainty = CARFREEDOM_VUNC;
	context.PulsesPer360 = CARFREEDOM_PULSES_PER_360;
	context.SteeringEncoderMaxLeft = CARFREEDOM_MAX_LEFT_STEERING_ENCODER;
	context.SteeringEncoderMaxRight = CARFREEDOM_MAX_RIGHT_STEERING_ENCODER;
	context.Encoder_FrontLeft_Invert = 0;
	context.Encoder_FrontRight_Invert = 0;
	context.Encoder_RearLeft_Invert = 0;
	context.Encoder_RearRight_Invert = 0;
	context.PrintCanFrames = 0;
	context.WriteCommandsRPH = 1;
	context.EnableMotion = 1;
}*/

void IniciaContext (Context &context){
	ros::NodeHandle nh("~");
	nh.param<int>("CarFreedom_Config_CarFreedom_CAN_Delay_Between_Queries_us", context.CAN_Delay_Between_Queries_us, 0);
	nh.param<int>("CarFreedom_Config_CarFreedom_Encoder_Pulses_Period_ms", context.Encoder_Pulses_Period_ms, 20);
	nh.param<double>("CarFreedom_Config_CarFreedom_MaxSteerAngleLeft", context.MaxSteerAngleLeft, CARFREEDOM_MAX_STEERING_ANGLE_DEG);
	nh.param<double>("CarFreedom_Config_CarFreedom_MaxSteerAngleRight", context.MaxSteerAngleRight, CARFREEDOM_MAX_STEERING_ANGLE_DEG);
	//nh.param<int>("CarFreedom_Config_CarFreedom_SteeringAng2CmdEnd1stLine_Adeg_Cmd", context.SteeringAng2CmdEnd1stLine_Adeg_Cmd, Ang2Cmd_End1stLine_);
	//nh.param<int>("CarFreedom_Config_CarFreedom_SteeringAng2CmdStart2ndLine_Adeg_Cmd", context.SteeringAng2CmdStart2ndLine_Adeg_Cmd, Ang2Cmd_Start2ndLine_);
	nh.param<double>("CarFreedom_Config_CarFreedom_SteeringAng2Cmd_ZeroCmd", context.SteeringAng2Cmd_ZeroCmd, 512);
	nh.param<double>("CarFreedom_Config_CarFreedom_MaxForwardSpeed", context.MaxForwardSpeed, CARFREEDOM_MAX_SPEED_FORWARD_M_S);
	nh.param<double>("CarFreedom_Config_CarFreedom_MaxReverseSpeed", context.MaxReverseSpeed, CARFREEDOM_MAX_SPEED_REVERSE_M_S);
	nh.param<double>("CarFreedom_Config_CarFreedom_JoystickMaxForward", context.JoystickMaxForward, CARFREEDOM_MAX_JOYSTICK_FORWARD);
	nh.param<double>("CarFreedom_Config_CarFreedom_JoystickMaxReverse", context.JoystickMaxReverse, CARFREEDOM_MAX_JOYSTICK_REVERSE);
	nh.param<double>("CarFreedom_Config_CarFreedom_JoystickMaxLeft", context.JoystickMaxLeft, CARFREEDOM_MAX_JOYSTICK_LEFT);
	nh.param<double>("CarFreedom_Config_CarFreedom_JoystickMaxRight", context.JoystickMaxRight, CARFREEDOM_MAX_JOYSTICK_RIGHT);
	nh.param<double>("CarFreedom_Config_CarFreedom_WheelRadius_m", context.WheelRadius_m, CARFREEDOM_WHEEL_RADIUS_M);
	nh.param<double>("CarFreedom_Config_CarFreedom_InterAxesDistance_m", context.InterAxesDistance_m, CARFREEDOM_INTER_AXES_DISTANCE_M);
	nh.param<double>("CarFreedom_Config_CarFreedom_AxisLength_m", context.AxisLength_m, CARFREEDOM_AXIS_LENGTH_M);
	nh.param<double>("CarFreedom_Config_CarFreedom_Wheels_V_Uncertainty", context.Wheels_V_Uncertainty, CARFREEDOM_VUNC);
	nh.param<double>("CarFreedom_Config_CarFreedom_PulsesPer360", context.PulsesPer360, CARFREEDOM_PULSES_PER_360);
	nh.param<double>("CarFreedom_Config_CarFreedom_SteeringEncoderMaxLeft", context.SteeringEncoderMaxLeft, CARFREEDOM_MAX_LEFT_STEERING_ENCODER);
	nh.param<double>("CarFreedom_Config_CarFreedom_SteeringEncoderMaxRight", context.SteeringEncoderMaxRight, CARFREEDOM_MAX_RIGHT_STEERING_ENCODER);
	nh.param<int>("CarFreedom_Config_CarFreedom_Encoder_FrontLeft_Invert", context.Encoder_FrontLeft_Invert, 0);
	nh.param<int>("CarFreedom_Config_CarFreedom_Encoder_FrontRight_Invert", context.Encoder_FrontRight_Invert, 0);
	nh.param<int>("CarFreedom_Config_CarFreedom_Encoder_RearLeft_Invert", context.Encoder_RearLeft_Invert, 0);
	nh.param<int>("CarFreedom_Config_CarFreedom_Encoder_RearRight_Invert", context.Encoder_RearRight_Invert, 0);
	nh.param<int>("CarFreedom_Config_CarFreedom_PrintCanFrames", context.PrintCanFrames, 0);
	nh.param<int>("CarFreedom_Config_CarFreedom_WriteCommandsRPH", context.WriteCommandsRPH, 1);
	nh.param<int>("CarFreedom_Config_CarFreedom_EnableMotion", context.EnableMotion, 1);
} //IniciaContext

Driver::Driver(Context context, socketcan_lib::Context socket_context) : CANDriver(socket_context,context.PrintCanFrames) {
  // the context is already saved in CanDriver
  //hydroutil::Properties &prop = context_.properties();

  // list of queries that must be sent at each period. Un/comment to send or not to send a query.
  queries2send.push_back(ID_QUERY_SUPERVISOR);
  //queries2send.push_back(ID_QUERY_MOTOR_LEFT);
  //queries2send.push_back(ID_QUERY_MOTOR_RIGHT);
  //queries2send.push_back(ID_QUERY_STEERING_CTRL);
  //queries2send.push_back(ID_QUERY_STEERING_ENCODER);
  //queries2send.push_back(ID_QUERY_ENCODER_REAR_RIGHT);
  //queries2send.push_back(ID_QUERY_ENCODER_REAR_LEFT);
  //queries2send.push_back(ID_QUERY_ENCODER_FRONT_RIGHT);
  //queries2send.push_back(ID_QUERY_ENCODER_FRONT_LEFT);

  querydelay_ = context.CAN_Delay_Between_Queries_us;
  encoder_pulses_period = context.Encoder_Pulses_Period_ms;

  // by default, capabilities are set to defines.
  capabilities_.maxSteeringAngleLeft = DEG2RAD(context.MaxSteerAngleLeft);
  capabilities_.maxSteeringAngleRight = DEG2RAD(context.MaxSteerAngleRight);

  Ang2Cmd_End1stLine_.clear(); Ang2Cmd_End1stLine_.resize(2);
  Ang2Cmd_End1stLine_[0]=0; Ang2Cmd_End1stLine_[1]=512;
  Ang2Cmd_End1stLine_ = context.SteeringAng2CmdEnd1stLine_Adeg_Cmd;
  Ang2Cmd_Start2ndLine_.clear(); Ang2Cmd_Start2ndLine_.resize(2);
  Ang2Cmd_Start2ndLine_[0]=0; Ang2Cmd_Start2ndLine_[1]=512;
  Ang2Cmd_Start2ndLine_ = context.SteeringAng2CmdStart2ndLine_Adeg_Cmd;
  // convert deg to radians!!!
  Ang2Cmd_End1stLine_[0]=DEG2RAD(Ang2Cmd_End1stLine_[0]);
  Ang2Cmd_Start2ndLine_[0]=DEG2RAD(Ang2Cmd_Start2ndLine_[0]);

  ZeroSteerCmd_ = context.SteeringAng2Cmd_ZeroCmd;

  capabilities_.maxForwardSpeed = context.MaxForwardSpeed;
  capabilities_.maxReverseSpeed = context.MaxReverseSpeed;

  capabilities_.maxSteeringAngleAtMaxSpeed=MIN(DEG2RAD(CARFREEDOM_MAX_STEERING_ANGLE_AT_MAX_SPEED_DEG),MIN(fabs(capabilities_.maxSteeringAngleRight),fabs(capabilities_.maxSteeringAngleLeft))-0.0001f);

  // but they may be redefined by properties
  cfg_.joy.maxforward=context.JoystickMaxForward;
  cfg_.joy.maxreverse=context.JoystickMaxReverse;
  cfg_.joy.maxleft=context.JoystickMaxLeft;
  cfg_.joy.maxright=context.JoystickMaxRight;
  // the vehicle geometry also can be redefined by properties
  cfg_.wheel_radius_m=context.WheelRadius_m;
  cfg_.inter_axis_m=context.InterAxesDistance_m;
  cfg_.axis_length=context.AxisLength_m;


  cfg_.encoder_Vunc=context.Wheels_V_Uncertainty;


  cfg_.pulses_per_360=context.PulsesPer360;

  cfg_.max_left_steering_encoder=context.SteeringEncoderMaxLeft;
  cfg_.max_right_steering_encoder=context.SteeringEncoderMaxRight;

  cfg_.encoder_frontleft_invert = context.Encoder_FrontLeft_Invert;
  cfg_.encoder_frontright_invert = context.Encoder_FrontRight_Invert;
  cfg_.encoder_rearleft_invert = context.Encoder_RearLeft_Invert;
  cfg_.encoder_rearright_invert = context.Encoder_RearRight_Invert;

	requery_now ();

  int verb=context.PrintCanFrames;
  setVerbose(0!=verb);

  cfg_.write_cmd_rph = context.WriteCommandsRPH;

  enableMotion_ = context.EnableMotion;

  encoders_ = new EncodersModel(cfg_);
  lastcmd_steeringAngle=lastcmd_motorspd=0;//no command has been sent, start with 0.
  last_tstamp_ = -1; /// -1 = invalid time stamp. no data received
	ROS_INFO("VEROCARFREEDOM: Driver iniciado!");
}//constructor

Driver::~Driver() {
  delete encoders_;
}

void Driver::enable() {
  network_start();
  setBaudRate(baudrate_); // needed if the usb plug is disconnected and then recconencted
  cerr << endl << " enable1 " << endl;
  // configure the motor encoders
  can_frame fr;
  fill_encoder_cfg_rear_right(fr,encoder_pulses_period);
  writeCanFrame(fr);
  // for each encoder, its response must be monitored, if it fails the message will be resent
  cansafe.addFrameToBeMonitored(ID_DATA_ENCODER_PULSES_REAR_RIGHT,fr);
  fill_encoder_cfg_rear_left(fr,encoder_pulses_period);
  writeCanFrame(fr);
  cansafe.addFrameToBeMonitored(ID_DATA_ENCODER_PULSES_REAR_LEFT,fr);
  fill_encoder_cfg_front_right(fr,encoder_pulses_period);
  writeCanFrame(fr);
  cansafe.addFrameToBeMonitored(ID_DATA_ENCODER_PULSES_FRONT_RIGHT,fr);
  fill_encoder_cfg_front_left(fr,encoder_pulses_period);
  writeCanFrame(fr);
  cansafe.addFrameToBeMonitored(ID_DATA_ENCODER_PULSES_FRONT_LEFT,fr);
}

/**
 * \brief sends all N queries and then tries to read N answers. if exacly N answers are given it will be fast with no waiting for time-outs
 * \par if readCAN() returns succesfully but less than N answers are read, then it tries again to complete N answers.
 * \par if readCAN() time-outs or gets another error it gives up even if there are less than N answers. It supposes that all queries will be answered instantaneously
 */
void Driver::read( verocarfreedomdefs_msgs::CarData &data )
{
  data_=&data;
  data_->hasFaults=data_->hasWarnings=0; // default - ok, no warnings/faults
  data_->warnFaultReason=" "; // no message to be print out
  stringstream warnfaults; warnfaults << " hydrv::CarFreedom "; // the beggining of all warn/fault messages

  // send all query frames-------------.
  pendingAnswers.clear(); //empty the pendingAnswers store.
  if (requery_now_) {
  		for(vector<unsigned int>::iterator it = queries2send.begin()
    		; it != queries2send.end(); it++)
  		{
    		can_frame fr;
    		fill_empty_canframe(fr,*it);
    		pendingAnswers.insert(*it);// store the query id.
    		writeCanFrame(fr);
    		if (querydelay_) usleep(querydelay_);// there may be a delay between each query
  		}
      requery_now_=false;
  }
  //---- read can frames --------
  bool keep_reading = true;
  while (keep_reading) {
    unsigned int nframes=pendingAnswers.size();//0;
    // reads can frames until it gets a time out or an error, or nframes are read.
    // if nframes = 0, it will read all frames available until it gets a timeout
    int readerr=readCAN(nframes); // nframes will be overwritten with the number of frames actually read
    if ( pendingAnswers.empty() || (readerr<0) )
      keep_reading=false; //decode() will erase queries from pendingAnswers
  }//while

  //------ we have finished reading from the can bus - time to get the timestamp ------
  data_->timeStamp = ros::Time::now();
  double current_tstamp = data_->timeStamp.toSec();
  //------- timestamp set -----------

  // only updates odometry if both encoders have sent data.
  if ((0==pendingAnswers.count(ID_QUERY_ENCODER_REAR_LEFT)) &&
    (0==pendingAnswers.count(ID_QUERY_ENCODER_REAR_RIGHT))) {
    double dt = -1; // if no data is receved, use invalid timestamp
    if (last_tstamp_ > 0)
      dt = current_tstamp - last_tstamp_; // current - last timestamp
    encoders_->update_carfreedomdata(data_,dt);
  }

  //--- check if the monitoring class is missing some frame
  can_frame fr_answer;
  while (cansafe.missingFrame(fr_answer)>0) {
    // if some frame is missing, it returns a frame that must be resent.
    writeCanFrame(fr_answer);
  }

  //-------------------------------------------------------------------
  //------------- check for warnings/faults to be set ------------------
  //--------------------------------------------------------------------
  //--- do not set warning/faults lightly - the health funtion in the component      ---
  //--- will print these messages in the stderr, and set the component state.        ---
  //--- on the other way, if you want to print out a message because the data is bad,---
  //--- the easiest way is to use this infrastructure.                               ---
  /// check if nothing was read!
  if (queries2send.size()==pendingAnswers.size()) {
    // after sending all queries, nothing was read. the can bus may have a problem
    data_->hasWarnings=1;// it is better to ignore this
    warnfaults << "No answer for " << pendingAnswers.size() << " CAN Queries ";
  }
  /// check if there are missing encoder frames
  if ( data_->enc_missing[0]+data_->enc_missing[1]+data_->enc_missing[2]+data_->enc_missing[3]>0) {
    data_->hasWarnings=1;// it is better to ignore this
    warnfaults<< " Missing encoder data (" << (int)data_->enc_missing[0] << " " << (int)data_->enc_missing[1] << " " << (int)data_->enc_missing[2] << " " << (int)data_->enc_missing[3] << ") ";
  }

  // if there is some warn or fault, copy the string into the data_
  if ((data_->hasFaults)||(data_->hasWarnings))
    data_->warnFaultReason=warnfaults.str();
  //-------------------- warning/faults ------------------------
  last_tstamp_ = current_tstamp; // saves current_tstamp to next time.
  //-----------------------------------------------------------
}//read

void Driver::write(/* const */ verocarfreedomdefs_msgs::CarCommand& command ){
  float motorcmdleft,motorcmdright,steerangle;
  unsigned int rphr=0,rphl=0;

  // --------------- calculate steerAngle command given the angle ------------
  // the steer command does not change with the different types of motor command.
  // here it is a simple line interpolation to convert sterr angle to steer command
  interpolate(capabilities_.maxSteeringAngleLeft,capabilities_.maxSteeringAngleRight,command.steerAngle,
	       cfg_.joy.maxleft,cfg_.joy.maxright,steerangle);

  // debug variables - just print outs
//cerr << endl << " mxL " << RAD2DEG(capabilities_.maxSteeringAngleLeft) << " angE1 " << RAD2DEG(Ang2Cmd_End1stLine_[0]) << " cmd " <<  RAD2DEG(command.steerAngle) << endl;
    //cerr << " jL " << cfg_.joy.maxleft << " jR " << cfg_.joy.maxright << " " << Ang2Cmd_End1stLine_[1] << " JS2 " << Ang2Cmd_Start2ndLine_[1] <<endl;
  //cerr << "AngS2 "<< RAD2DEG(Ang2Cmd_Start2ndLine_[0]) << " mxR" << RAD2DEG(capabilities_.maxSteeringAngleRight);

  /*//------------------------------------------------------------
  //--- using different lines to interpolate left, right and dead zone when converting cmd to angle.
  if (command.steerAngle < Ang2Cmd_End1stLine_[0]) { //belong to the 1st line
    interpolate(capabilities_.maxSteeringAngleLeft,Ang2Cmd_End1stLine_[0],command.steerAngle,
		 cfg_.joy.maxleft,Ang2Cmd_End1stLine_[1],steerangle);
  } else if (command.steerAngle > Ang2Cmd_Start2ndLine_[0]) {//belong to the 2nd line
    interpolate(Ang2Cmd_Start2ndLine_[0],capabilities_.maxSteeringAngleRight,command.steerAngle,
		 Ang2Cmd_Start2ndLine_[1],cfg_.joy.maxright,steerangle);
  } else { // interpolate a 3rd line between the extremes of the two defined lines.
    interpolate(Ang2Cmd_End1stLine_[0],Ang2Cmd_Start2ndLine_[0],command.steerAngle,
	        Ang2Cmd_End1stLine_[1],Ang2Cmd_Start2ndLine_[1],steerangle);
  }
  // -------------  end thre line interpolation ---------------*/

  //cerr << " Int " << steerangle;
  //-----------------------  deal with motor commands ------------------------
  // prepare the data, transforming from speed [m/s] into the correct units and values
  if (cfg_.write_cmd_rph) { // if RPH commands are sent
    motorcmdleft=command.speedLeft; motorcmdright=command.speedRight;
    if (cfg_.write_cmd_rph>=2) { // if the differential is calculated HERE
      float stangle = steeringCmd2Angle(steerangle);
      differentialDrive(stangle, motorcmdright,motorcmdleft);
    }
    // transforming commands into rph
    rphr = mps2rph(motorcmdright / 3.6f );// the command is in km/h
    rphl = mps2rph(motorcmdleft / 3.6f ); // the command is in km/h

  } else {
    interpolate(-capabilities_.maxReverseSpeed,capabilities_.maxForwardSpeed,command.speedLeft,
		cfg_.joy.maxreverse,cfg_.joy.maxforward,motorcmdleft);
    interpolate(-capabilities_.maxReverseSpeed,capabilities_.maxForwardSpeed,command.speedRight,
		cfg_.joy.maxreverse,cfg_.joy.maxforward,motorcmdright);
  }

  //------------------------------------------------------------------
  // actually fill the can frame structure and send the data on the bus
  can_frame f;
  if (enableMotion_) {
    int res=0;
    if (cfg_.write_cmd_rph==1) {
      fill_motordrv_cmd_rph(f,(rphr+rphl)/2);// one can frame, with a unique forward speed
      res+=writeCanFrame(f);
    } else if (cfg_.write_cmd_rph >= 2) {
      fill_motordrv_cmd_rph(f,rphr,rphl);// one can frame, with 2 motor speeds
      res+=writeCanFrame(f);
    } else {
	fill_motordrv_cmd_right(f,(unsigned int)motorcmdright, 0);
	res+=writeCanFrame(f);
	fill_motordrv_cmd_left(f,(unsigned int)motorcmdleft, 0);
	res+=writeCanFrame(f);
    }//if (cfg_.write_cmd_rph)

    fill_steering_cmd(f,(unsigned int) steerangle);
    res+=writeCanFrame(f);
  }//if enableMotion_

  // --- save the last values sent on the bus.
  lastcmd_steeringAngle=(unsigned int) steerangle;
  if (cfg_.write_cmd_rph) {
    lastcmd_motorspd= (rphr + rphl)/2;
  } else {
    lastcmd_motorspd=((unsigned int)motorcmdleft+(unsigned int)motorcmdright)/2;
  }
}//write

void Driver::getStatus( std::string &status, bool &isWarn, bool &isFault ){

}

/**
 * \brief converts from meters per second to rotations per hour
 * \par this must be done here, as the parameters values may be changed with Driver .cfg options
 */
unsigned int Driver::mps2rph(float speed_mps) {
  return (unsigned int) (speed_mps * 3600 / (CARFREEDOM_PI * 2 * cfg_.wheel_radius_m));
}//mps2rph

/**
 * \brief convert the steering command to actual angle values.
 * \par by now, using a simple linear regression based on the measurements of 100104
 * @TODO it has NOT been updated to new measurements because it is used only to generate differential, it is not beeing used now.
 * @param stcmd it is the joystick command (0-1024) and not the angle
 * @return steering angle in radians
 */
float Driver::steeringCmd2Angle(float stcmd) {
  float anglel = DEG2RAD(-0.03827 * stcmd + 18.0595); // R2 = 0.9932
  float angler = DEG2RAD(-0.04065 * stcmd + 22.4124); // R2 = 0.9692;
  return (anglel+angler)/2; // as the wheels are not well aligned, return the average between both of them.
}//steeringCmd2Angle


/**
* \brief differential drive model. Uses the stering angle to change the individual wheel commands
* \par equation: VD = VE (1+ (axis_length/ 2 inter_axis_m)*tan(steerAngle) / (1-(axis_length/2 inter_axis_m)*tan(steerAngle))
* @return updated commands for right and left wheels. The input units do not matter, it only applys a ratio.
*/
void Driver::differentialDrive(float stangle, float &cmdr, float &cmdl) {
  //cfg_.inter_axis_m = L ;  cfg_.axis_length = D
  // Josue equation: VD = VE (1+ (D/2L)*tan(phi) / (1- (D/2L)*tan(phi))

  // calculate proportion between left and right
  float d2ltphi = (tan(stangle)*cfg_.axis_length)/(2*cfg_.inter_axis_m) ;
  float ratio = (1 + d2ltphi) / (1 - d2ltphi);
  // now, supposing cmdr and cmdl are equal, calculate an increment that will make cmdr = cmdl * ratio;
  float increment = - cmdr * (1-ratio)/(1+ratio);
  // now update cmdr and cmdl
  cmdr -= increment;
  cmdl += increment;
}//differentialDrive


/**
 * \brief decodes can frames, reimplements from CanDriver
 * @uses calls a function from canfreedom_defs to extract data from each type of can frame
 */
int Driver::decode(struct can_frame frame) {
  CAN_decode_error canerr= CANFRAME_OK;
  unsigned char encoder_counter=0;
  unsigned char encoder_error=0; // encoder error is currently not utilized
  unsigned int pulses=0;
  int speed, angle;
  CANDriver::decode(frame); // print it out, just during tests
  switch (frame.can_id) {
    case ID_DATA_SUPERVISOR:
		pendingAnswers.erase(ID_QUERY_SUPERVISOR);
		canerr=extractSupervisorFrame(frame,*data_);
      break;
    case ID_CMD_MOTOR_JOYSTICK:
      canerr=extractJoystickDataSpeed (frame, speed);
		ROS_INFO ("got ID_CMD_MOTOR_JOYSTICK! speed: %d", speed);
		interpolate(cfg_.joy.maxforward, cfg_.joy.maxreverse, speed, capabilities_.maxForwardSpeed, -capabilities_.maxReverseSpeed, data_->referenceSpeed);
      break;
    case ID_CMD_STEERING:
      canerr=extractJoystickDataAngle (frame, angle);
		interpolate(cfg_.joy.maxleft, cfg_.joy.maxright, angle, capabilities_.maxSteeringAngleLeft, capabilities_.maxSteeringAngleRight, data_->referenceSteerAngle);
      break;
    case ID_DATA_CORRENTE_POTENCIA:
      canerr=extractCorrentePotenciaData (frame, *data_);
      break;
    case ID_DATA_BATERIA:
      canerr=extractBateriaData (frame, *data_);
      break;
    case ID_DATA_MOTOR_LEFT:
      pendingAnswers.erase(ID_QUERY_MOTOR_LEFT);
      canerr=extractDataMotorFrame(frame,*data_);
      break;
    case ID_DATA_MOTOR_RIGHT:
      pendingAnswers.erase(ID_QUERY_MOTOR_RIGHT);
      canerr=extractDataMotorFrame(frame,*data_);
      break;
    case ID_DATA_STEERING_CTRL:
      pendingAnswers.erase(ID_QUERY_STEERING_CTRL);
      canerr=extractDataSteeringCtrlFrame(frame,*data_);
      break;
    case ID_DATA_STEERING_ENCODER:
      pendingAnswers.erase(ID_QUERY_STEERING_ENCODER);
      canerr=extractDataSteeringEncoderFrame(frame,*data_);
      // convert from encoder postion to angle in radians
      double aux;
      interpolate(cfg_.max_left_steering_encoder,cfg_.max_right_steering_encoder,data_->steerAngle,
		  capabilities_.maxSteeringAngleLeft,capabilities_.maxSteeringAngleRight,aux);
      data_->steerAngle=aux;
      encoders_->new_steeringAngle(aux);
      break;
    case ID_DATA_ENCODER_FRONT_LEFT:
      pendingAnswers.erase(ID_QUERY_ENCODER_FRONT_LEFT);
      canerr=extractDataMotorEncoderFrame(frame,*data_);
      break;
    case ID_DATA_ENCODER_FRONT_RIGHT:
      pendingAnswers.erase(ID_QUERY_ENCODER_FRONT_RIGHT);
      canerr=extractDataMotorEncoderFrame(frame,*data_);
      break;
    case ID_DATA_ENCODER_REAR_LEFT:
      pendingAnswers.erase(ID_QUERY_ENCODER_REAR_LEFT);
      canerr=extractDataMotorEncoderFrame(frame,*data_);
      break;
    case ID_DATA_ENCODER_REAR_RIGHT:
      pendingAnswers.erase(ID_QUERY_ENCODER_REAR_RIGHT);
      canerr=extractDataMotorEncoderFrame(frame,*data_);
      break;
    case ID_DATA_ENCODER_PULSES_FRONT_LEFT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,encoder_counter,pulses);
      encoders_->new_encoder_reading(FRONT_LEFT,pulses,encoder_counter,encoder_error);
      cansafe.gotFrame(frame.can_id); // the monitoring class
      break;
    case ID_DATA_ENCODER_PULSES_FRONT_RIGHT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,encoder_counter,pulses);
      encoders_->new_encoder_reading(FRONT_RIGHT,pulses,encoder_counter,encoder_error);
      cansafe.gotFrame(frame.can_id); // the monitoring class
      break;
    case ID_DATA_ENCODER_PULSES_REAR_LEFT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,encoder_counter,pulses);
      encoders_->new_encoder_reading(REAR_LEFT,pulses,encoder_counter,encoder_error);
      cansafe.gotFrame(frame.can_id); // the monitoring class
      break;
    case ID_DATA_ENCODER_PULSES_REAR_RIGHT:
      canerr=extractDataMotorEncoderPulsesFrame(frame,encoder_counter,pulses);
      encoders_->new_encoder_reading(REAR_RIGHT,pulses,encoder_counter,encoder_error);
      cansafe.gotFrame(frame.can_id); // the monitoring class
      break;
  }//switch
  return 0;
}//decode


verocarfreedom::Capabilities Driver::capabilities() const {
    return capabilities_;
}



} // namespace
/*
extern "C" {
    hydrointerfaces::CarFreedomFactory *createCarDriverFactory()
    { return new hydrocarfreedom::Factory; }
}

*/

