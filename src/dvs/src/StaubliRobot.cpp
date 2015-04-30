#include "StaubliRobot.h"

using namespace cv;


namespace dvs {

StaubliRobot::StaubliRobot(ros::NodeHandle &nh):RobotProxy(nh){ 
  sock_=NULL; readParam();}

int StaubliRobot::readParam() {
    //get parameters for socket. this has a client, the robot has the server.
    int port; std::string host;
    nh_.param<std::string>("staubli/host",host,"localhost");
    nh_.param("staubli/port",port,1111);
    // if we already have a socket, close it
    if (sock_) {
      sock_->close();
      delete sock_;
    }
    // do not need SocketFactory, this already knows it is a Staubli
    sock_ = new sockutil::StaubliClient(host,port);
    if (!sock_) {
	ROS_ERROR("socket is NULL. Are you sure the robot parameter is right?");
	return -1;
    } else {
      if(sock_->connect()) {
	ROS_ERROR("the socket can not connect at host %s:%d : ",sock_->getHost().c_str(),sock_->getPort());//,sock_->errmsg().c_str());
      }
    }
    
    // get parameters for robot geometry (transformation camera -> robot)
    nh_.param("staubli/theta_deg",theta_,-20.0); /// only tilt currently 
    double x,y,z; /// translation camera->robot
    nh_.param("staubli/rTc/x",x, 0.0);
    nh_.param("staubli/rTc/y",y,-0.05);
    nh_.param("staubli/rTc/z",z,+0.05);
    Tc_ = (Mat_<double>(3,1) << x, y, z); 
    return 0;
}//readParam

/**
 * convert the camera velocities to robot velocities and send the robot velocities from dvsstat to the robot
 */
int StaubliRobot::commRobot(esmocv::Dvs_Status &dvsstat) {
  transCam2Rob(dvsstat);
  if (!sock_) {
    ROS_ERROR("socket is NULL, but you tried to send data. Are you sure the robot parameter is right?");
    return -1;
  }
  if (sock_->send(dvsstat.v_r)) {
    ROS_ERROR_THROTTLE(5,"socket error on send");
  }
  if (sock_->rec(dvsstat.v_r_mes,dvsstat.pose)) {
    ROS_ERROR_THROTTLE(5,"socket error on receive");
  }
  return 0;
}//commRobot

/// a zero velocity command is sent to the robot.
int StaubliRobot::stopRobot(esmocv::Dvs_Status &dvsstat) {
  //transCam2Rob(dvsstat);
  // zero the robot velocity
  dvsstat.v_r[0]=dvsstat.v_r[1]=dvsstat.v_r[2]=dvsstat.v_r[3]=dvsstat.v_r[4]=dvsstat.v_r[5]=0.0f;
  if (!sock_) {
    ROS_ERROR("socket is NULL, but you tried to stop the robot. Are you sure the robot parameter is right?");
    return -1;
  }
  std::vector<double> vel_zero(6,0.0);// 6 elements with zero value
  if (sock_->send(vel_zero)) {
    ROS_ERROR_THROTTLE(5,"socket error: sending ZERO command");
  }
  if (sock_->rec(dvsstat.v_r_mes,dvsstat.pose)) {
    ROS_ERROR_THROTTLE(5,"socket error: receive after ZERO command");
  }
  return 0;
}//stopRobot


/**
 * \brief PURPOSE : to transfer the velocities from the camera frame to the robot frame. The considered robot is the TX90 arm from Staubli.
 * \par INPUT  
 * \li v_c = [v_x, v_y, v_z, w_x, w_y, w_z]^T, where z-axis is aligned to the optical axis and y-axis is pointed downwards [m/s]
 * \li theta is the elevation angle of the camera relative to the z-axis of the TCP. Negative value upwards [deg].
 * \li tc = [t_x, t_y, t_z]^T is the translation of the camera frame w.r.t. center of the TCP. Axis are similar to the camera [m].
 * \par OUTPUTS 
 * \li v_r = v_x, v_y, v_z, w_x, w_y, w_z]^T in R^6 represents, respectively, the translational [mm/s] and rotational velocities [deg/s] of the robotic arm.
 * 
 * \par AUTHOR  : Geraldo Silveira
 * \par http://sites.google.com/site/geraldofsilveira
 * \par Geraldo.Silveira@cti.gov.br
 */
int StaubliRobot::transCam2Rob(esmocv::Dvs_Status &dvsstat) {
  /* orignal matlab code
function v_r = camVel2robVel_staubli(v_c, theta, tc)
% Particular modifications to the Staubli case (-90 deg rotation in z-axis)
% Rotation of -theta degs in y-axis and -90 degs in z-axis (w.r.t. robot frame),
% i.e, (R_y*R_z)
rRc = [0 cosd(theta) -sind(theta); -1 0 0; 0 sind(theta) cosd(theta)];
% Translation is relative to a -90 deg rotation in z-axis
rtc = [0 1 0; -1 0 0; 0 0 1]*tc;

% Velocity transformation
v_r = [rRc ppv(rtc)*rRc; zeros(3,3) rRc] * v_c;

% Adjustment of translational velocities to the Staubli arm
v_r(1:3) = v_r(1:3) *1000; % mm/s
v_r(4:6) = v_r(4:6) *180/pi; % deg/s

   */ 
  Mat rRc = Mat::zeros(3, 3, CV_64F); // init 
  double thetarad = utDEG2RAD(theta_); 
  rRc.at<double>(0,1) = cos(thetarad);
  rRc.at<double>(0,2) = -sin(thetarad);
  rRc.at<double>(1,0) = -1;
  rRc.at<double>(2,1) = sin(thetarad);
  rRc.at<double>(2,2) = cos(thetarad);
  
  Mat t_axes = (Mat_<double>(3,3) <<  0, 1, 0,
				      -1, 0, 0,
				       0, 0, 1);
  Mat rtc = t_axes * Tc_;

  /////% Velocity transformation
  /////v_r = [rRc ppv(rtc)*rRc; zeros(3,3) rRc] * v_c; 
  Mat vel_trans = Mat::zeros(6, 6, CV_64F);
  rRc.copyTo(vel_trans(cv::Rect(0,0,3,3)));// rect(x,y,width,height) 
  rRc.copyTo(vel_trans(cv::Rect(3,3,3,3)));// rect(x,y,width,height) 
  Mat ppvrtcrRc = dvsutil::Ut::ppv(rtc) * rRc;
  ppvrtcrRc.copyTo(vel_trans(cv::Rect(3,0,3,3)));// rect(x,y,width,height)
  Mat v_c = Mat::zeros(6, 1, CV_64F);
  dvsutil::Ut::copyArray2Mat(dvsstat.v_c,v_c); // get v_c from message

  Mat v_r = vel_trans * v_c;

  ////// unit conversion/////////////////////
  // convert m/s in mm/s (*1000) and rad/s in deg/s (180/M_PI)
  Mat mulVR = (Mat_<double>(6,1) <<  1000,1000,1000,
				      180.0/M_PI,180.0/M_PI,180.0/M_PI);
  v_r = v_r.mul(mulVR);// per element multiplication
  
  
  ////////////// copy back into dvstat
  dvsutil::Ut::copyMat2Array(v_r,dvsstat.v_r); // write v_r into message
  
}//transCam2Rob

} ; //namespace