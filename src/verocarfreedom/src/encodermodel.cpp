
#include "encodermodel.h"
#include <cv.h>
#include <math.h>



using namespace std;
using namespace verocarfreedomdefs;

namespace verocarfreedom
{

  EncodersModel::EncodersModel(verocarfreedom::Config &cfg) : cfg_(cfg) {
  for (int i=0; i<4 ; i++) { // zeroing everything
    last_encoder_counters[i]=0;
    last_encoder_pulses[i]=0;
    missingframes[i]=0;
    deltapulses[i]=0;
    initialized[i]=false;
  }
  // zeroing the odometry.
  odom_.x= 0;
  odom_.y= 0;
  odom_.theta  = 0;
} // constructor


/**
* \brief checks the counters from the encoder data, and if there are missing packets sets the appropriate flags. Call this function imediately after each can frame is decoded.
* @par note that the counters are a byte field that overflows in 255...
* @param i 0 frontleft, 1 front right, 2 rear left, 3 rear right
*/
void EncodersModel::check_encoder_counters(WHEEL i,unsigned char encoder_counter,unsigned char encoder_error) {

  if (!initialized[i]) {
    // it is the first data packet! no sense to compare with anything.
    last_encoder_counters[i]=encoder_counter;
    missingframes[i]=0;
    return;
  }
  // yes, if last == 255, then the sum will overflow and we get 0. it will be correct if you keep the types as unsigned char.
  if (encoder_counter!=last_encoder_counters[i]+1) {
    // Houston, we have a problem. we've lost an encoder can frame.

    char missing; // how many can_frames were lost and are misssing?
    if (encoder_counter<last_encoder_counters[i])// check if overflow...
      missing = (255 - last_encoder_counters[i])+encoder_counter;
    else // if there were not overflow
      missing = encoder_counter - last_encoder_counters[i] - 1;
    missingframes[i]+=missing;
  } else { // no can frame has been lost.
    // do nothing, it is fine
  }
  // anyway, the new value will be saved for the next cycle
  last_encoder_counters[i] = encoder_counter;
}// check_encoder_counters


void EncodersModel::absolute2delta(WHEEL i,unsigned int pulses, unsigned char encoder_counter,unsigned char encoder_error) {

  if (!initialized[i]) {
    // it is the first data packet! no sense to compare with anything.
    last_encoder_pulses[i]=pulses;
    deltapulses[i]=0;
    return;
  }

  const unsigned int MAXINT = 65535;
  const unsigned int uplimit =   2*MAXINT/3;
  const unsigned int downlimit = MAXINT/3;

  // check if there were overflow
  if ( (last_encoder_pulses[i] > uplimit) && ( pulses < downlimit) ) {
    // overflow from max to 0.
    deltapulses[i] +=  (MAXINT - last_encoder_pulses[i] + 1) + pulses;
  } else if ( (last_encoder_pulses[i] < downlimit) && ( pulses > uplimit) ) {
    // overflow from 0 to max; it is a subtraction (-=)
    deltapulses[i] -= (MAXINT - pulses + 1) + last_encoder_pulses[i];
  } else if (pulses > last_encoder_pulses[i]) { // no overflow, normal operation, going up
    deltapulses[i] += pulses - last_encoder_pulses[i];
  } else if (pulses <= last_encoder_pulses[i]){ // no overflow, normal operation, going down
    deltapulses[i] -= last_encoder_pulses[i] - pulses;
  }
  //cerr << " abs[ " << i << " p " << pulses << " " << deltapulses[i] <<" ] ";
  last_encoder_pulses[i]=pulses; // copy: the current will be the last one next time.
}//encoder_absolute2delta


/**
* \brief converts raw encoder data into odometry readings.
* \par here is the implementation of the odometry model
* \par odometry: x = east, y = north, in meters ; o in radians, zero in the start.
*/


void EncodersModel::updateOdometry() {

	int i,j;
  // calculate d_theta in radians

  float d_thetaL = deltapulses[REAR_LEFT] * 2.0f * CARFREEDOM_PI / (float)cfg_.pulses_per_360;
  float d_thetaR = deltapulses[REAR_RIGHT] * 2.0f * CARFREEDOM_PI / (float)cfg_.pulses_per_360;
  float d_thetaL_F = deltapulses[FRONT_LEFT] * 2.0f * CARFREEDOM_PI / (float)cfg_.pulses_per_360;
  float d_thetaR_F = deltapulses[FRONT_RIGHT] * 2.0f * CARFREEDOM_PI / (float)cfg_.pulses_per_360;

  //------ check if we have to revert the encoders.

  //-- some encoders are mounted such that negative readings means the car moves forward

  if (cfg_.encoder_rearleft_invert)
    d_thetaL = - d_thetaL;

  if (cfg_.encoder_rearright_invert)
    d_thetaR = - d_thetaR;

  if (cfg_.encoder_frontleft_invert)
    d_thetaL_F = - d_thetaL_F;

  if (cfg_.encoder_frontright_invert)
    d_thetaR_F = - d_thetaR_F;


  // calculate linear wheel displacement

  float ql = d_thetaL * cfg_.wheel_radius_m;
  float qr = d_thetaR * cfg_.wheel_radius_m;
  float ql_f = d_thetaL_F * cfg_.wheel_radius_m;
  float qr_f = d_thetaR_F * cfg_.wheel_radius_m;

  // state reconstruction

  double eR = cfg_.axis_length/2;
  double L = cfg_.inter_axis_m;
  //rmartins
  //double q1 = cfg_.encoder_Vunc;
  double q1 = 0.02;
  double q2 = 0.05;
  double sigmal = atan(tan(stAng_)/(1-eR*tan(stAng_)/L));
  double sigmar = atan(tan(stAng_)/(1+eR*tan(stAng_)/L));

   cv::Mat C(5,2,CV_64FC1);  

	C.at<double>(0,0) = tan(stAng_);
	C.at<double>(0,1) = -L;
	
	for (i=1;i<5;i++) {
		C.at<double>(i,0) = 1;
		C.at<double>(i,1) = eR*pow(-1,i-1);
	}

    cv::Mat W(5,5,CV_64FC1);  

	for (i=0;i<5;i++) {
		for (j=0;j<5;j++) {
			if (i==j && i==0) 	W.at<double>(i,j) = 1/(pow(q2,2));
			else if (i==j)		W.at<double>(i,j) = 1/(pow(q1,2));
			else 			W.at<double>(i,j) = 0;
		}
	}

    cv::Mat z(1,5,CV_64FC1); 
	
	z.at<double>(0,0) = 0;
	z.at<double>(0,1) = qr;
	z.at<double>(0,2) = ql;
	z.at<double>(0,3) = qr_f*cos(sigmar);
	z.at<double>(0,4) = ql_f*cos(sigmal);

	cv::Mat aux(2,2,CV_64FC1);
	aux = C.t()*W*C;
	
	cv::Mat fg(2,1,CV_64FC1);
	fg = aux.inv(cv::DECOMP_SVD)*(C.t())*W*(z.t());

	Ddt_ = fg.at<double>(0,0);
	Dth_ = fg.at<double>(1,0);
  odom_.x+= Ddt_ * cos(odom_.theta+Dth_/2.0f); // cos = 1 if theta = 0:
  odom_.y+= Ddt_ * sin(odom_.theta+Dth_/2.0f); // sin = 1 if theta = 90:
  odom_.theta  += Dth_; // in radians

}//updateOdometry

/**
 * \brief called when a new encoder reading is received. Updates the internal pulse counters.
 * @param errcode Currently not utilized. Use 0
 */
void EncodersModel::new_encoder_reading(WHEEL w,unsigned int pulses,unsigned char counter, unsigned char errcode) {

  check_encoder_counters(w,counter,errcode);
  absolute2delta(w,pulses,counter,errcode);

  if (!initialized[w]) {
    initialized[w]=true;
    //cerr << endl << " ************** init! " << w << " **************** " << endl;
  }
}//new_encoder_reading


/**
* \brief the information stored in this class is written in a CarFreedom::Data.
* @param data_ must be previously allocated. write speeds, encoders, odometry
* @param dt time, in seconds, between the last call of this function and now.
*/
void EncodersModel::update_carfreedomdata(verocarfreedomdefs_msgs::CarData *data_, double dt) {
  //cerr <<" box "<< odom_.p.x << " p [" <<deltapulses[REAR_LEFT] << "," << deltapulses[REAR_RIGHT] << "]";
  updateOdometry();
  //cerr<< " aox "<< odom_.p.x << " p [" <<deltapulses[REAR_LEFT] << "," <<deltapulses[REAR_RIGHT] << "]";
  // save raw pulses into data. if there are missing frames, speed is divided by the number of missing frames..
  data_->speedLeft = deltapulses[REAR_LEFT] ;// (missingframes[REAR_LEFT]+1);
  data_->speedRight = deltapulses[REAR_RIGHT];/// (missingframes[REAR_RIGHT]+1);
  data_->speedLeftFront = deltapulses[FRONT_LEFT];/// (missingframes[FRONT_LEFT]+1);
  data_->speedRightFront = deltapulses[FRONT_RIGHT];/// (missingframes[FRONT_RIGHT]+1);

  //------ check if we have to revert the encoders. some encoders are mounted such that negative readings means that the car is moving forward
  if (cfg_.encoder_rearleft_invert)
    data_->speedLeft = - data_->speedLeft;
  if (cfg_.encoder_rearright_invert)
    data_->speedRight = - data_->speedRight;
  if (cfg_.encoder_frontright_invert)
    data_->speedRightFront = - data_->speedRightFront;
  if (cfg_.encoder_frontleft_invert)
    data_->speedLeftFront = - data_->speedLeftFront;

  // save odom into data_
  data_->x    = odom_.x;
  data_->y    = odom_.y;
  data_->yaw  = odom_.theta;
  if (dt>0) {
  data_->vlong = Ddt_ / dt; // instantaneous linear speed [m/s]
  data_->dyaw = Dth_ / dt; // instantaneous angular speed [rad/s]
  } else {
    data_->vlong = data_->dyaw = 0; // with invalid delta time, zero speed
  }
  for (int i=0; i < 4 ; i++) {// saves missing frames
    data_->enc_missing[i]=missingframes[i];
  }

  for (int i=0; i<4 ; i++) { // zeroing reported data
    missingframes[i]=0;
    deltapulses[i]=0;
  }

}//update_carfreedomdata


}// namespace
