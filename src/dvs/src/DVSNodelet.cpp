
#include "DVSNodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(dvs, DVSNodelet, dvs::DVSNodelet, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS(ddvs, DDVSNodelet, dvs::DDVSNodelet, nodelet::Nodelet);


namespace dvs
{

/**
 * \brief subscribes/advertises and initialize
 * \par there are issues with nodehandles in nodelets. Just to create a nodelet does not get the right namespace. It is necessary to get a private nodehandle **after** the object is created. This means that attribute classes which need a nodehandle in the appropriate namespace can not be just object atributes, because they can not be initialized in the constructor. 
 * \par this is why esm is a pointer, and why control and robot inside the esm class are also pointers. The nodehandle only can exist after the object is initialized.
 */
CTRLNodelet::CTRLNodelet()// : nh_(),it_(nh_) 
  {
    got_track = false;
    ctrl_success_ = false;
  
    G = Mat::zeros(3,3,CV_64F);
  }//constructor
  
CTRLNodelet::~CTRLNodelet() { }// destructor
 
/**
 * \brief get private nodehandle, to be able to initialize esm and its children.
 * \par from abstract nodelet class
 * \par note: only subscribe at the end of this function to avoid callbacks running before everything is initialized
 */
void CTRLNodelet::onInit()
  {
   NODELET_INFO("DVSNodelet::onInit ");
   dvsutil::Ut::openCVStatus();
   //private_nh.getParam("value", value_);
   nhPriv_ = getPrivateNodeHandle();
   nh_ = getNodeHandle();
   
  /// //// get specific robot
  std::string robotname;/// used to choose the robotproxy class, which handles the transformation of camera and robot frames and communicates with the robot.
  nhPriv_.param<std::string>("robot",robotname,"staubli");
  RobotFactory *rf_ = RobotFactory::uniqueInstance();
  robot_.reset(rf_->newRobot(robotname,nhPriv_));
  
   ///  we need to get the ROI from the track parameters
   ROIupdated();

   ctrl_.reset(createCtrl());
   
   /// ///// subscribe and advertise stuff
   dvs_pub = nh_.advertise<esmocv::Dvs_Status>("dvs_status", 100);
   trk_sub = nh_.subscribe("track_status", 1,&DVSNodelet::TrkCallback,this);
   
   NODELET_INFO("DVSNodelet initialized with robot %s",robotname.c_str());
  }//onInit
  
  
/**
 * \brief sets a value for the K matrix based on parameters. 
 */
void CTRLNodelet::fillBogusCameraInfo() {
  
  double focal_length; int imageWidth,imageHeight;
  nhPriv_.param("/esmocv/focal_length_pix",focal_length,801.0);
  nhPriv_.param("/esmocv/image_width",imageWidth,960);
  nhPriv_.param("/esmocv/image_height",imageHeight,1280);

  K = (Mat_<double>(3,3) << focal_length, 0, imageWidth/2.0, 
			    0, focal_length, imageHeight/2.0,
			    0, 0, 1);
  
} //fillBogusCameraInfo

/// \brief when ROI is set, some constants must be recalculated.
void CTRLNodelet::ROIupdated() {
  ///  we need to get the ROI from the track parameters
  nhPriv_.getParam("/esmocv/roi/x", ROI.x);
  nhPriv_.getParam("/esmocv/roi/y", ROI.y);
  nhPriv_.getParam("/esmocv/roi/width", ROI.width);
  nhPriv_.getParam("/esmocv/roi/height", ROI.height);
  //G0 = [1,0,winPos(1); 0,1,winPos(2); 0,0,1];
  G0 = (Mat_<double>(3,3) <<1, 0, ROI.x, 
		  0, 1, ROI.y,
		  0, 0, 1)  ;
  //pt = [winPos(1)+round(winPos(3)/2); winPos(2)+round(winPos(4)/2); 1];
  pt = (Mat_<double>(3,1) << ROI.x + round(ROI.width/2.0),
			     ROI.y + round(ROI.height/2.0),
			      1);
}//ROIupdated
  
void CTRLNodelet::TrkCallback(const nodelet::ESMTrack &msg) {
  
    //NODELET_INFO_THROTTLE(10,"dvs trk cbck: RMS %.2f ",msg.RMS);
    esmocv::Dvs_Status dvsstat; /// message to be sent

    fpsutil_.cycleHZ(dvsstat.fps_cur); // calculate fps
    
    if (!got_track) {
      got_track = true;
      ROIupdated();
      fillBogusCameraInfo();// have to wait until tracking is running, or we will not have the correct image size.
      std::stringstream ss;
      ss << "\nROI " << ROI ;
      ss << "\nK " << K ;
      NODELET_INFO("DVSNodelet got track msg, parameters: %s",ss.str().c_str());
    }//if
    
    // copy msg.H into G.
    G.at<double>(0,0) = msg.H[0];
    G.at<double>(0,1) = msg.H[1];
    G.at<double>(0,2) = msg.H[2];
    G.at<double>(1,0) = msg.H[3];
    G.at<double>(1,1) = msg.H[4];
    G.at<double>(1,2) = msg.H[5];
    G.at<double>(2,0) = msg.H[6];
    G.at<double>(2,1) = msg.H[7];
    G.at<double>(2,2) = msg.H[8];
    
    //ptw(:,iter) = Gc*pt; ptw(:,iter) = ptw(:,iter)./repmat(ptw(3,iter),3,1);
    Gc = G * (G0.inv());
    /////////// set timestamp ////////////
    dvsstat.header.stamp = ros::Time::now();
    
    // do the control here...
    if (ctrl_) {
      ctrl_->doCtrl(Gc, K, pt, dvsstat);//e, v_c);
      ctrl_success_ = ctrl_->stopCondReached();
    } else 
      ROS_ERROR("control NULL in dvsnodelet.");
    
    /// call the robot to transform velocity and send command
    if (robot_) // if robot is null, do not call it!!!
      if ((msg.stop) || 
	  ctrl_success_ || // if the ctrl has been successfull, stop the robot 
	  (!ctrl_)) {// if there is no ctrl, stop the robot too.
	NODELET_INFO_THROTTLE(5,"ROBOT STOPPED: %s %s"
		      ,(msg.stop?"TRACKING STOPPED;":"")
		      ,(ctrl_success_?"CTRL SUCCESSFUL!":""));
	robot_->stopRobot(dvsstat);
      } else  
	robot_->commRobot(dvsstat);
    
    dvs_pub.publish(dvsstat);
    
  }//TrkCallback
  
/// callback from camera. use it if the nodelet needs images... not now.
void CTRLNodelet::ImgCallback(const sensor_msgs::ImageConstPtr& msg) {
  /*
  NODELET_INFO_THROTTLE(10,"dvs callback: #%ld image %d x %d",fpsutil_.getCount(),msg->width,msg->height);
    
  // cv bridge, convert ros message in opencv object
  cv_bridge::CvImageConstPtr cv_ptr;
  try  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);//SHARED!!!
  } catch (cv_bridge::Exception& e) {
    NODELET_ERROR("cv_bridge exception: %s", e.what());
    return;
  }//try
  
  cv::imshow("DVS nodelet", cv_ptr->image);//debug
  cv::waitKey(1); // needed for imshow http://stackoverflow.com/questions/5217519/opencv-cvwaitkey
  
  esmocv::Dvs_Status dvsstatmsg;
  dvsstatmsg.fps_cur = 1;// writes the fps on the message
  
  
  dvs_pub.publish(dvsstatmsg);
  
  
  */
}// callback
  
  
}//namespace


