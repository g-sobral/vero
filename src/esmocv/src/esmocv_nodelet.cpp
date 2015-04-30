
#include "esmocv_nodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(esmocv, ESMOCVNodelet, esmocv::ESMOCVNodelet, nodelet::Nodelet);


namespace esmocv
{

/**
 * \brief subscribes/advertises and initialize
 * \par there are issues with nodehandles in nodelets. Just to create a nodelet does not get the right namespace. It is necessary to get a private nodehandle **after** the object is created. This means that attribute classes which need a nodehandle in the appropriate namespace can not be just object atributes, because they can not be initialized in the constructor. 
 * \par this is why esm is a pointer, and why control and robot inside the esm class are also pointers. The nodehandle only can exist after the object is initialized.
 */
ESMOCVNodelet::ESMOCVNodelet()  {
  
  WINDOW = "ESMOCVNodelet";// just a name for OCV window
  initHasRun = false;
  pub_every_frame_=0;
  count_frames_=0;
  //cv::namedWindow(WINDOW);
  
}//constructor
  
ESMOCVNodelet::~ESMOCVNodelet() {
  //cv::destroyWindow(WINDOW);
  if (procThread_running_) {
    NODELET_INFO("shutting down processing thread");
    procThread_running_ = false;
    procThread_->join();
    NODELET_INFO("processing thread stopped");
  }
}// destructor
  
  
  bool ESMOCVNodelet::restartROI(esmocv::SetROI::Request& req,esmocv::SetROI::Response& ans) {
    NODELET_INFO("ESMOCVNodelet::restart ROI (%d,%d) w %d h %d dorect %d",
		 req.roi.x_offset,req.roi.y_offset,req.roi.width,
		 req.roi.height,req.roi.do_rectify);
    ans.error = 0;// it is ok!
    
    ros::Time tstamp = ros::Time::now();
    
    if (esm_) esm_->closeOCVWindowOnDelete(false); // to faster restart, leave windows open
       
    /// change the ROI parameters
    nhPriv_.setParam("roi/x", (int) req.roi.x_offset); // setPAram has NO Uint version, cast to int
    nhPriv_.setParam("roi/y", (int) req.roi.y_offset);
    nhPriv_.setParam("roi/width",(int) req.roi.width);
    nhPriv_.setParam("roi/height",(int) req.roi.height);
    esm_.reset(new ESMOCV(nhPriv_)); // create a new one. smartptr guarantees the previous one is deleted
    
    ros::Duration elapsed = ros::Time::now() - tstamp;
    NODELET_INFO( "SRV END ESMOCVNodelet::restartROI %.3f ms",elapsed.toSec()*1000.0);
    waitForRestartROI_ = 0 ;/// we got a ROI, so we are not waiting for it anymore
    return true; // ok!
  } // restartROI
 
/**
 * \brief get private nodehandle, to be able to initialize esm and its children.
 */
void ESMOCVNodelet::onInit()
  { NODELET_INFO("ESMOCVNodelet::onInit ");
   
  nh_ = getNodeHandle(); /// nodelets use this API to get nodehandles
  nhPriv_ = getPrivateNodeHandle();
    
  // with the nodehandle, we can init the image transport
  it_.reset(new image_transport::ImageTransport(nh_));

  //dvs_pub = nh_.advertise<esmocv::Dvs_Status>("dvs_status", 100);
  trk_pub = nh_.advertise<nodelet::ESMTrack>("track_status", 100);
  
  // service to restart 
  servSetROI = nh_.advertiseService("restartROI", &ESMOCVNodelet::restartROI,this); 
  
  nhPriv_.param("waitForRestartROI",waitForRestartROI_, 0);
  if (!waitForRestartROI_){ 
    esm_.reset(new ESMOCV(nhPriv_));
  }
  // find name of image topic and subscribe
  std::string imagetopicname; 
  nh_.param<std::string>("image_topic_name",imagetopicname,"/ptgrey_simple/pgr_mono");
  
  // advertise slower image topic, if needed
  nhPriv_.param("publish_slower_image",pub_every_frame_, 0);
  if (pub_every_frame_) {
    // set name of topic to publish slower images
    slower_image_topic_ = imagetopicname+"_slow"; 
    imageSlow_pub_ = it_->advertise(slower_image_topic_, 1);
  }
 
  // if you do not want a separate thread to process images, 
  // call ImgCallback instead of RawImgCallback
  // but I do not recommend that.
  NODELET_INFO("ESMOCVNodelet::onInit subscribing to %s ", imagetopicname.c_str());
  image_sub_ = it_->subscribe(imagetopicname, 1, &ESMOCVNodelet::RawImgCallback, this);
  
  // spawn processing thread
  procThread_running_ = true;
  procThread_ = boost::shared_ptr< boost::thread > (new boost::thread(boost::bind(&ESMOCVNodelet::ProcImgThread, this)));
  
  initHasRun = true;
}//onInit
  
/** \brief callback direct from camera. Only stores images. A separate consumer thread processes the images
 * \par implementation of consumer-producer, quite more hardcore than needed, lets go in the safe side
 * 
 * \par for info about the consumer-producer sync problem:
 * \see http://en.wikipedia.org/wiki/Producer%E2%80%93consumer_problem
 * \see http://en.wikipedia.org/wiki/Semaphore_%28programming%29#Semaphore_vs._mutex
 */
void ESMOCVNodelet::RawImgCallback(const sensor_msgs::ImageConstPtr& msg) {
  mutex_empty.lock(); // block if it is not empty (it means the consumer is getting data)
  mutex_shared.lock(); //block before acessing critical region 
  //NODELET_INFO("Raw: sha lock === msg=%i msgIn__seq=%i",(msg?msg->header.seq:-1),
  //		                                 (msgIn_?msgIn_->header.seq:-1));
  msgIn_ = msg; 
  mutex_shared.unlock(); // release critical region
  mutex_full.unlock(); // release full lock, the consumer may go ahead
}// the raw, direct camera image callback

/** \brief thread to process images. actual processing is done in the ImgCallback function
 * \par implements a separate thread to call ImgCallback when a new image is stored by RawImgCallback
 */
void ESMOCVNodelet::ProcImgThread(void) {
  
  while (procThread_running_) {
      mutex_full.lock(); // block if it is not full (there is no image)
      mutex_shared.lock(); // block if other thread is acessing the critical region
      //NODELET_INFO("proc: sha lock === msgIn_seq=%i msgIn2__seq=%i",(msgIn_?msgIn_->header.seq:-1),(msgIn2?msgIn2->header.seq:-1));
      
      if ( (msgIn_) /*&& // msgin must exist
	 ( (!msgIn2) // if msgIn2 is NULL, it is the 1st time. 
	   || (msgIn_->header.seq != msgIn2->header.seq))*/) 
         { // the seq number must have changed = new image
	// new image
	msgIn2 = msgIn_; // shallow copy, just transfers ownership
	mutex_shared.unlock(); // release critical region
	mutex_empty.unlock(); // release empty lock (producer can continue)
	// do not invert the order of these lines 
	ImgCallback(msgIn2); /// this must be out of the critical region! 
      } else {
	// if we decided not to use the image, we release the locks nevertheless
	mutex_shared.unlock(); 
	mutex_empty.unlock();
      }
  }//while
}//procThread
  
  
/// processing work is done here. get images from RawImgCallback
void ESMOCVNodelet::ImgCallback(const sensor_msgs::ImageConstPtr& msg) {
  
  if (!initHasRun) { NODELET_WARN_THROTTLE(1,"esmocv callback: onInit() has not run !!!"); return; }
  if (!esm_){
    NODELET_WARN_THROTTLE(1,"esmocv callback: ESM object is not initialized !!!"); 
    //if (!waitForRestartROI_) // if we are not waiting, it is a problem!
    //  esm_.reset(new ESMOCV(nhPriv_)); // try to restart it!
    return;// do nothing if we have not a esm tracker!  
  } 
  //NODELET_ERROR("CB BEG");
  NODELET_INFO_THROTTLE(10,"esmocv callback: image %d x %d",msg->width,msg->height); 
  
  //////////// cv bridge, convert ros message in opencv object
  cv_bridge::CvImageConstPtr cv_ptr;
  try  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);//SHARED!!!
  } catch (cv_bridge::Exception& e) {
    NODELET_ERROR("image call back: cv_bridge exception: %s", e.what());
    return;
  }//try
  
  
  nodelet::ESMTrack trackstatmsg;
  
  //cv::imshow(WINDOW, cv_ptr->image);//debug
  //cv::waitKey(1); // needed for imshow http://stackoverflow.com/questions/5217519/opencv-cvwaitkey
  cv::Mat H = cv::Mat::zeros(3,3,CV_64F);;
  //ros::Time before = ros::Time::now();
    
  esm_->procImg(cv_ptr->image,H,trackstatmsg);
  
  /// a smart pointer is needed to nodelet inter-process publishing
  nodelet::ESMTrackPtr trackstatmsg_p(new nodelet::ESMTrack(trackstatmsg));
  trk_pub.publish(trackstatmsg_p);
  
  //////////// publish in slow image topic (DEBUG and RESULT FOR PAPER
  if (pub_every_frame_) {
    if ((count_frames_ % pub_every_frame_)==0) { 
      cv_bridge::CvImage bridgedImage;
      bridgedImage.header = msg->header;
      bridgedImage.encoding = sensor_msgs::image_encodings::MONO8;
      bridgedImage.image = esm_->getSmallImage(); 
      imageSlow_pub_.publish(bridgedImage.toImageMsg());
    }//endif of publish slow image topic  
  }
  count_frames_++; // always increment even if it do not publish
  
  
  //NODELET_ERROR("CB END");
}// callback
  
  /// convenience function to print nicely a small matrix
  void ESMOCVNodelet::printMat(std::string msg,const Mat &M) {
   std::stringstream ss;
   ss << std::setw( 10 ) << std::setprecision( 3 ) << msg << M ;
   NODELET_INFO_THROTTLE(1,"%s",ss.str().c_str());
  }
  
}//namespace

  
  /*  format of the image ros message
    $ rosmsg show sensor_msgs/Image 
    Header header
    uint32 seq
    time stamp
    string frame_id
    uint32 height
    uint32 width
    string encoding
    uint8 is_bigendian
    uint32 step
    uint8[] data
    */
