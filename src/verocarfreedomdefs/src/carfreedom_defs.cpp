#include "carfreedom_defs.h"

namespace verocarfreedomdefs
{

/**
 * \brief fill a empty can frame with the given can_id. Use this to generate can_frames to query the system.
 * @param f the can frame to be filled. all 8 data bytes will be overwritten
 * @ID the CAN ID to be set
 */
void fill_empty_canframe(struct can_frame &f, unsigned int ID) {
  f.can_id = ID;
  f.can_dlc=1;
  f.data[0]=1;f.data[1]=f.data[2]=f.data[3]=f.data[4]=f.data[5]=f.data[6]=f.data[7]=0;
};//fill,empty_canframe


CAN_decode_error extractSupervisorFrame(struct can_frame &frame,verocarfreedomdefs_msgs::CarData &data) {
    if (frame.can_id!=ID_DATA_SUPERVISOR) return CANFRAME_BADID;
    if (frame.can_dlc!=4) return CANFRAME_BADLENGTH;

    data.bumperRear  = (frame.data[0]&0x01);
    data.bumperFront = (frame.data[0]&0x02);
    data.battVolt = ((frame.data[1] + (frame.data[2]*256))*0.0383)+12.98;
    data.mode = (frame.data[3] & 0b00110000)>>4;
    data.CarOn = (frame.data[3] & 0x08);
    data.buzz =  (frame.data[3] & 0x04);
    data.brake = (frame.data[3] & 0x02);
    data.strobo= (frame.data[3] & 0x01);
    return CANFRAME_OK;
};

CAN_decode_error extractDataMotorFrame(struct can_frame &frame,verocarfreedomdefs_msgs::CarData &data) {
    if ((frame.can_id!=ID_DATA_MOTOR_LEFT)&&(frame.can_id!=ID_DATA_MOTOR_RIGHT))
      return CANFRAME_BADID;
    if (frame.can_dlc!=7) return CANFRAME_BADLENGTH;

    float ampere = ((frame.data[0] + (frame.data[1]*256))+(frame.data[2] + (frame.data[3]*256)) )*0.488f;
    if (ID_DATA_MOTOR_LEFT==frame.can_id) {
      data.ampereMotorLeft=ampere;
      data.acelLeft = frame.data[4];
    } else if (ID_DATA_MOTOR_RIGHT==frame.can_id) {
      data.ampereMotorRight=ampere;
      data.acelRight = frame.data[4];
    } else {
      return CANFRAME_BADID;
    }

  return CANFRAME_OK;
};//extractDataMotorFrame

CAN_decode_error extractDataSteeringCtrlFrame(struct can_frame &frame,verocarfreedomdefs_msgs::CarData &data) {
  if (frame.can_id!=ID_DATA_STEERING_CTRL)
    return CANFRAME_BADID;
  if (frame.can_dlc!=3) return CANFRAME_BADLENGTH;

  data.ampereSteering = (frame.data[0] + (frame.data[1]*256))/8.53f;

  data.endCourseSteering = (frame.data[2]!=0);

  return CANFRAME_OK;
};//extractDataSteeringCtrlFrame

CAN_decode_error extractDataSteeringEncoderFrame(struct can_frame &frame,verocarfreedomdefs_msgs::CarData &data) {
  if (frame.can_id!=ID_DATA_STEERING_ENCODER)
    return CANFRAME_BADID;
  if (frame.can_dlc!=2) return CANFRAME_BADLENGTH;

  int ang = (frame.data[0] + (frame.data[1]*256));
  data.steerAngle=ang; ////the caller should convert from encoder range to angle!!!

  return CANFRAME_OK;
};//extractDataSteeringEncoderFrame


CAN_decode_error extractDataMotorEncoderFrame(struct can_frame &frame,verocarfreedomdefs_msgs::CarData &data) {
  if ((frame.can_id!=ID_DATA_ENCODER_FRONT_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_FRONT_RIGHT)&&
    (frame.can_id!=ID_DATA_ENCODER_REAR_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_REAR_RIGHT))
    return CANFRAME_BADID;
  if (frame.can_dlc!=5) return CANFRAME_BADLENGTH;
  float *ff = ((float*)frame.data);
  if (ID_DATA_ENCODER_REAR_LEFT==frame.can_id) {
    data.speedLeft=*ff;
  }
  if (ID_DATA_ENCODER_REAR_RIGHT==frame.can_id) {
    data.speedRight=*ff;
  }
  if (ID_DATA_ENCODER_FRONT_LEFT==frame.can_id) {
    data.speedLeftFront=*ff;
  }
  if (ID_DATA_ENCODER_FRONT_RIGHT==frame.can_id) {
    data.speedRightFront=*ff;
  }
  //data.bicData.vlong=*ff; used to test.
  return CANFRAME_OK;
};//extractDataMotorEncoderFrame

/**
 * @param counter note this extra parameter corresponding to the extra field marca in the can frame. The driver should check the counter and set the warning flags and fields on its data accordingly.
 */
CAN_decode_error extractDataMotorEncoderPulsesFrame(struct can_frame &frame, unsigned char &counter,
						     unsigned char &enc_err, verocarfreedomdefs_msgs::CarData &data) {
  if ((frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_RIGHT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_RIGHT))
    return CANFRAME_BADID;
  if (frame.can_dlc!=4) return CANFRAME_BADLENGTH;
  int pulses = (frame.data[0] + (frame.data[1]*256));
  counter = frame.data[2];
  enc_err = frame.data[3];
  if (ID_DATA_ENCODER_PULSES_REAR_LEFT==frame.can_id) {
    data.speedLeft=pulses;
  }
  if (ID_DATA_ENCODER_PULSES_REAR_RIGHT==frame.can_id) {
    data.speedRight=pulses;
  }
  if (ID_DATA_ENCODER_PULSES_FRONT_LEFT==frame.can_id) {
    data.speedLeftFront=pulses;
  }
  if (ID_DATA_ENCODER_PULSES_FRONT_RIGHT==frame.can_id) {
    data.speedRightFront=pulses;
  }
  return CANFRAME_OK;
};//extractDataMotorEncoderFrame

CAN_decode_error extractDataMotorEncoderPulsesFrame(struct can_frame &frame, unsigned char &counter,
						     verocarfreedomdefs_msgs::CarData &data) {
  if ((frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_RIGHT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_RIGHT))
    return CANFRAME_BADID;
  if (frame.can_dlc!=3) return CANFRAME_BADLENGTH;
  int pulses = (frame.data[0] + (frame.data[1]*256));
  counter = frame.data[2];
  if (ID_DATA_ENCODER_PULSES_REAR_LEFT==frame.can_id) {
    data.speedLeft=pulses;
  }
  if (ID_DATA_ENCODER_PULSES_REAR_RIGHT==frame.can_id) {
    data.speedRight=pulses;
  }
  if (ID_DATA_ENCODER_PULSES_FRONT_LEFT==frame.can_id) {
    data.speedLeftFront=pulses;
  }
  if (ID_DATA_ENCODER_PULSES_FRONT_RIGHT==frame.can_id) {
    data.speedRightFront=pulses;
  }
  return CANFRAME_OK;
};//extractDataMotorEncoderFrame



/**
* @param counter note this extra parameter corresponding to the extra field marca in the can frame. The driver should check the counter and set the warning flags and fields on its data accordingly.
*/
CAN_decode_error extractDataMotorEncoderPulsesFrame(struct can_frame &frame, unsigned char &counter,
						     unsigned char &enc_err,
						     unsigned int &pulses) {
  if ((frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_RIGHT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_RIGHT))
    return CANFRAME_BADID;
  if (frame.can_dlc!=4) return CANFRAME_BADLENGTH;
  pulses = (frame.data[0] + (frame.data[1]*256));
  counter = frame.data[2];
  enc_err = frame.data[3];

  return CANFRAME_OK;
};//extractDataMotorEncoderFrame
CAN_decode_error extractDataMotorEncoderPulsesFrame(struct can_frame &frame, unsigned char &counter,
						     unsigned int &pulses) {
  if ((frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_FRONT_RIGHT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_LEFT)&&
    (frame.can_id!=ID_DATA_ENCODER_PULSES_REAR_RIGHT))
    return CANFRAME_BADID;
  if (frame.can_dlc!=3) return CANFRAME_BADLENGTH;
  pulses = (frame.data[0] + (frame.data[1]*256));
  counter = frame.data[2];

  return CANFRAME_OK;
};//extractDataMotorEncoderFrame

CAN_decode_error extractJoystickDataSpeed (struct can_frame frame, int &speed) {
	if (frame.can_id != ID_CMD_MOTOR_JOYSTICK) return CANFRAME_BADID;
	if (frame.can_dlc!=5) return CANFRAME_BADLENGTH;
	short aux;
	memcpy((void*)&aux,(void*)&frame.data[0],2);
	speed = (int)aux;

	return CANFRAME_OK;
}//extractJoystickDataSpeed

CAN_decode_error extractJoystickDataAngle (struct can_frame frame, int &angle) {
 	if (frame.can_id != ID_CMD_STEERING) return CANFRAME_BADID;
	if (frame.can_dlc!=4) return CANFRAME_BADLENGTH;
	short aux;
	memcpy((void*)&aux,(void*)&frame.data[0],2);
	angle = (int)aux;

	return CANFRAME_OK;
} // extractJoystickData

CAN_decode_error extractCorrentePotenciaData (struct can_frame frame, verocarfreedomdefs_msgs::CarData &data_){
 	if (frame.can_id != ID_DATA_CORRENTE_POTENCIA) return CANFRAME_BADID;
	if (frame.can_dlc!=8) return CANFRAME_BADLENGTH;
	short aux1;
	memcpy((void*)&aux1,(void*)&frame.data[0],2);
	data_.ampereMotorLeft = aux1;
	memcpy((void*)&aux1,(void*)&frame.data[2],2);
	data_.ampereMotorRight = aux1;
	memcpy((void*)&aux1,(void*)&frame.data[4],2);
	data_.Potenciaesq = aux1;
	memcpy((void*)&aux1,(void*)&frame.data[6],2);
	data_.Potenciadir = aux1;

	return CANFRAME_OK;
} // extractCorrentePotenciaData

CAN_decode_error extractBateriaData (struct can_frame frame, verocarfreedomdefs_msgs::CarData &data_){
 	if (frame.can_id != ID_DATA_BATERIA) return CANFRAME_BADID;
	if (frame.can_dlc!=4) return CANFRAME_BADLENGTH;
	char aux1;
	short aux2;
	memcpy((void*)&aux1,(void*)&frame.data[0],1);
	data_.ErroEsq = aux1;
	memcpy((void*)&aux1,(void*)&frame.data[1],1);
	data_.ErroDir = aux1;

	return CANFRAME_OK;
} // extractBateriaData


//--------------------------------------------------------------------------------------
// ---------------------   FUNCTIONS TO FILL CAN FRAMES TO BE SENT TO THE CAR ----------
//--------------------------------------------------------------------------------------



int fill_supervisor_cmd(struct can_frame &frame,char modo, char lights, char buz) {
  frame.can_id = ID_CMD_SUPERVISOR;
  frame.can_dlc = 3;
  if (modo>3) return -1;
  if (modo<1) return -1;
  frame.data[0] = modo;
  frame.data[1] = (lights!=0);
  frame.data[2] = (buz!=0);
  return 0;
}//fill_supervisor_cmd

/**
 * \brief usually it will be called with just cmd and brk parameters. Moreover, usually the helper functions for the right and left motors will be used.
 * @param status only used during calibration. usually, this parameter is not needed, by default it will be zero
 * @param ch_liga not clear if it is read at all. usually, this parameter is not needed, by default it will be zero
 */
int fill_motordrv_cmd(unsigned int id,struct can_frame &frame,unsigned int cmd, char ch_brk, char status, char ch_liga) {
  frame.can_id = id;
  frame.can_dlc = 5;
  memcpy((void*)frame.data,(void*)&cmd,2);
  frame.data[2] = (ch_brk!=0);
  frame.data[3] = (status!=0);
  frame.data[4] = (ch_liga!=0);
  return 0;
}//fill_supervisor_cmd
int fill_motordrv_cmd_right(struct can_frame &frame,unsigned int cmd, char ch_brk, char status, char ch_liga) {
  return fill_motordrv_cmd(ID_CMD_MOTOR_RIGHT,frame,cmd,ch_brk,status,ch_liga);  }
int fill_motordrv_cmd_left(struct can_frame &frame,unsigned int cmd, char ch_brk, char status, char ch_liga) {
  return fill_motordrv_cmd(ID_CMD_MOTOR_LEFT,frame,cmd,ch_brk,status,ch_liga);  }



int fill_motordrv_cfgAccel(unsigned int id,struct can_frame &frame,char accel) {
  frame.can_id = id;
  frame.can_dlc = 1;
  frame.data[0] = accel;
  return 0;
}//fill_motordrv_cfgAccel
int fill_motordrv_cfgAccel_left(struct can_frame &frame,char accel){
  return fill_motordrv_cfgAccel(ID_CFG_MOTOR_LEFT,frame,accel);}
int fill_motordrv_cfgAccel_right(struct can_frame &frame,char accel){
  return fill_motordrv_cfgAccel(ID_CFG_MOTOR_RIGHT,frame,accel);}


int fill_motordrv_cmd_rph(struct can_frame &frame, unsigned int rph_r, unsigned int rph_l) {
  frame.can_id = ID_CMD_MOTOR_RPH;
  frame.can_dlc = 5;
  frame.data[0] = 0; // two speed mode
  memcpy((void*)&frame.data[1],(void*)&rph_r,2); // copy the ints into the can frame.
  memcpy((void*)&frame.data[3],(void*)&rph_l,2);
  return 0;
}

int fill_motordrv_cmd_rph(struct can_frame &frame, unsigned int rph_f) {
  frame.can_id = ID_CMD_MOTOR_RPH;
  frame.can_dlc = 3;
  frame.data[0] = 1; // a unique forward speed mode
  memcpy((void*)&frame.data[1],(void*)&rph_f,2); // copy the ints into the can frame.
  return 0;
}

/**
 * \brief usually it will be called with just cmd parameter.
 * @param ch_liga not clear if it is read at all. usually, this parameter is not needed, by default it will be zero
 * @param ch_brk not clear if it is read at all. usually, this parameter is not needed, by default it will be zero
 */
int fill_steering_cmd(struct can_frame &frame,unsigned int cmd, char ch_liga, char ch_brk) {
  frame.can_id = ID_CMD_STEERING;
  frame.can_dlc = 2;
  memcpy((void*)frame.data,(void*)&cmd,2);
  return 0;
}//fill_steering_cmd


/**
* \brief generic function to configure the wheel encoders. usually you will call the other functions which call this one
* @param period all values are valid. if > 0, defines the period, if 0, disables the periodic encoder counter packet.
*/
int fill_encoder_cfg(unsigned int id,struct can_frame &frame,unsigned char period) {
  frame.can_id = id;
  frame.can_dlc = 1;
  frame.data[0] = period;
  return 0;
}//fill_encoder_cfg
/**
* \brief configures rear right wheel encoder
*/
int fill_encoder_cfg_rear_right(struct can_frame &frame,unsigned char period) {
  return fill_encoder_cfg(ID_CFG_ENCODER_PERIOD_REAR_RIGHT,frame,period);
}//fill_encoder_cfg
/**
* \brief configures rear left wheel encoder
*/
int fill_encoder_cfg_rear_left(struct can_frame &frame,unsigned char period) {
  return fill_encoder_cfg(ID_CFG_ENCODER_PERIOD_REAR_LEFT,frame,period);
}//fill_encoder_cfg
/**
* \brief configures front right wheel encoder
*/
int fill_encoder_cfg_front_right(struct can_frame &frame,unsigned char period) {
  return fill_encoder_cfg(ID_CFG_ENCODER_PERIOD_FRONT_RIGHT,frame,period);
}//fill_encoder_cfg

/**
* \brief configures front left wheel encoder
*/
int fill_encoder_cfg_front_left(struct can_frame &frame,unsigned char period) {
  return fill_encoder_cfg(ID_CFG_ENCODER_PERIOD_FRONT_LEFT,frame,period);
}//fill_encoder_cfg


} //namespace
