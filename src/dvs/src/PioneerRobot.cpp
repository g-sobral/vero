#include "PioneerRobot.h"

using namespace cv;


namespace dvs {

PioneerRobot::PioneerRobot(ros::NodeHandle &nh):RobotProxy(nh){ 
  readParam();has_ptl_pose_ = false;}

int PioneerRobot::readParam() {
    
    // get parameters for robot geometry
    //nh_.param("pioneer/theta_deg",theta_,0.0);
    double x,y,z;
    nh_.param("pioneer/lambda_w",lambda_w_, 1.0);
    nh_.param("pioneer/rTc/x",x, 0.0);
    nh_.param("pioneer/rTc/y",y, 0.0);
    nh_.param("pioneer/rTc/z",z,-0.27);
    rTc_ = (Mat_<double>(3,1) << x, y, z);
    
    // init publishers and subscribers
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ptl_pub_ = nh_.advertise<geometry_msgs::Twist>("/PTU_cmd_vel", 100);
    ptlpose_sub_ = nh_.subscribe("/PTU46_pose", 100, &PioneerRobot::pantiltPoseCallback, this);
    
    return 0;
}//readParam

/**
 * \brief transforms camera velocity in robot and pantilt commands and publishes to robot and pantilt
 * \par if no pantilt pose has been received, it will stop robot and pantilt.
 */
int PioneerRobot::commRobot(esmocv::Dvs_Status &dvsstat) {
  if (!has_ptl_pose_) {
    ROS_ERROR_THROTTLE(5,"has not received pantilt pose, can not calculate pantilt speed, will stop robot");
    stopRobot(dvsstat);
    return -1;
  }
  transCam2Rob(dvsstat);
  
  // publish in smart pointer to allow nodelet interprocess communication
  geometry_msgs::TwistPtr msgptl(new geometry_msgs::Twist); 
  //geometry_msgs::Twist msgptl;
  msgptl->angular.y = tilt_speed_;
  msgptl->angular.z = pan_speed_;
  ptl_pub_.publish(msgptl);
  
  //geometry_msgs::Twist msgrobot;
  geometry_msgs::TwistPtr msgrobot(new geometry_msgs::Twist); 
  msgrobot->linear.x = robot_v_linear_;
  msgrobot->angular.z = robot_w_angular_;
  vel_pub_.publish(msgrobot);
  
  return 0;
}//commRobot


/**
 * \brief sends zeroed messages to both robot and pantilt, stopping both.
 */
int PioneerRobot::stopRobot(esmocv::Dvs_Status &dvsstat) {
  transCam2Rob(dvsstat);
  
  //geometry_msgs::Twist msgptl; // publish zeroed message
  geometry_msgs::TwistPtr msgptl(new geometry_msgs::Twist); 
  ptl_pub_.publish(msgptl);
  
  //geometry_msgs::Twist msgrobot; // publish zeroed message
  geometry_msgs::TwistPtr msgrobot(new geometry_msgs::Twist); 
  vel_pub_.publish(msgrobot);
  
  return 0;
}//stopRobot

/**
 * \brief PURPOSE : to transfer the velocities from the camera frame to the robot %           frame, considering the motion of the pan-tilt. A unicycle-type mobile robot is considered.
 * \par INPUT   
 * \li v_c = [v_x, v_z, w_x, w_y]^T in R^4 where z-axis is aligned to the optical axis and y-axis is pointed downwards
 * \par values needed 
 * \li theta_p is the current pan angle [radians/s]
 * \li theta_t is the current tilt angle [radians/s]
 * \li lambda_w > 0 is a rotation gain (due to a possibly too small d_p)
 * \li d_p is the z-distance [m] from the robot frame to the camera frame
 * \par OUPUT   
 * \par v_r = [v, w_r, w_p, w_t] respectively the translational and rotational velocities (robot, pan and tilt)   
 * \par AUTHOR  : Geraldo Silveira
 * \par http://sites.google.com/site/geraldofsilveira
 * \par Geraldo.Silveira@cti.gov.br
 */
int PioneerRobot::transCam2Rob(esmocv::Dvs_Status &dvsstat) {
  /* just to keep this information
  % length of pantilt arms, in meters, from the base of pantilt (not from the robot). 
   %armP = 0.08;
   %armT = 0.04; 
   %heightT = 0.06; % actually we do not need it, we care only about velocities
   */
  
  /* orignal matlab code
   
    W = [-sin(theta_p)/cos(theta_t), cos(theta_p)/cos(theta_t), 0, 0; ...
               cos(theta_p)/d_p,          sin(theta_p)/d_p, 0, 0; ...
               -cos(theta_p)/d_p,         -sin(theta_p)/d_p, 0, 1; ...
                              0,                         0, 1, 0];

    v_r = W * v_c;

    lambda_w=1;
    % Particular modifications to the Pioneer case
    %v_r(1) = v_r(1)  *100; % mm/s
    v_r(2) = -v_r(2) * lambda_w; % counterclockwise is positive
    v_r(3) = -v_r(3) * lambda_w; % conterclockwise is positive
    %v_r(4) = v_r(4)  *lambda_w ; % degrees/s
    
   */ 
  
  double d_p = rTc_.at<double>(2,0); /// shorthand: get the distance robot-cam in z plane
  
  tf::Quaternion qptl(lastptl_.orientation.x,lastptl_.orientation.y,lastptl_.orientation.z,lastptl_.orientation.w); 
  // so we use tf:: classes to convert to RPY angles
  double roll;
  double theta_t; // tilt is the pitch;
  double theta_p;  // pan is the yaw
  tf::Matrix3x3(qptl).getRPY(roll, theta_t, theta_p); // roll pitch yaw
  //ROS_DEBUG("RPY = (%lf, %lf, %lf)", utRAD2DEG(roll), utRAD2DEG(theta_t), utRAD2DEG(theta_p));
  
  Mat transCam2Rob = (Mat_<double>(4,6) << 
    -sin(theta_p)/cos(theta_t),0, cos(theta_p)/cos(theta_t),	0,0,0,
    cos(theta_p)/d_p ,         0, sin(theta_p)/d_p, 		0,0,0,
    -cos(theta_p)/d_p,         0, -sin(theta_p)/d_p, 		0,1,0,
    0,0,0,							1,0,0
    );
  
  Mat v_c = Mat::zeros(6, 1, CV_64F);
  dvsutil::Ut::copyArray2Mat(dvsstat.v_c,v_c); // get v_c from message
  
  Mat v_r_p = transCam2Rob * v_c; // vels of robot and pantilt
  /// v_r_p is not a nice vector. lets give some names to its positions
  /// we also fix units and signal conventions here
  robot_v_linear_ = v_r_p.at<double>(0,0);
  robot_w_angular_ = -v_r_p.at<double>(1,0) * lambda_w_; // counterclockwise is positive
  pan_speed_ = -v_r_p.at<double>(2,0)* lambda_w_ ; // counterclockwise is positive
  tilt_speed_ = v_r_p.at<double>(3,0)* lambda_w_ ;
  
  /// dvsstat.v_r is not going to be used, it is just for the record
  dvsstat.v_r[0] = robot_v_linear_; // copy into robot vx
  dvsstat.v_r[5] = robot_w_angular_; // copy into robot wz
  
}//transCam2Rob

/**
 * \brief ROS callback to receive the current pan and tilt angles from the pantilt driver 
 */
void PioneerRobot::pantiltPoseCallback(const geometry_msgs::Pose pos){
  has_ptl_pose_ = true;
  lastptl_ = pos;  
}//pantiltPoseCallback


} ; //namespace