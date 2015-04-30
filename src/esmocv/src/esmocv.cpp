
#include "esmocv.h"


namespace esmocv
{
  
  /**
   * \brief default constructor
   * 
   */
ESMOCV::ESMOCV(ros::NodeHandle &nh) : nh_(nh)  {
  stop_ = false;
  //ctrl = NULL; 
  //robot_= NULL;
  WINDOW="ESMOCV tracking";
  REFWINDOW="ESM Reference Image";
  
  ptw = Mat::zeros(3,1,CV_64F);
  
  fps_cur = 0;
  plot_every_frame=10;
  
  fillDefaults();
  fillBogusCameraInfo();
  
  hasIref=false;
  
  //http://stackoverflow.com/questions/15388781/cvnamedwindow-hangs-when-called
  //cv::namedWindow(WINDOW,CV_WINDOW_NORMAL|CV_GUI_NORMAL);
  //cv::namedWindow(REFWINDOW,CV_WINDOW_NORMAL|CV_GUI_NORMAL);
  //cv::waitKey(5);
  //cv::startWindowThread();
  closeWindowOnDelete = true;
  
  frameCount=0;
  
}// constructor

ESMOCV::~ESMOCV() {
  if (closeWindowOnDelete) {
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(REFWINDOW);
  }
}//destructor

 
/**
 * \brief sets a value for the K matrix
 * @param reads rosparam focal_length_pix (in pixels)
 */
void ESMOCV::fillBogusCameraInfo() {
  
  nh_.param("focal_length_pix",focal_length,500.0);
  K = (Mat_<double>(3,3) << focal_length, 0, imageWidth/2.0, 
			    0, focal_length, imageHeight/2.0,
			    0, 0, 1);
  
} //fillBogusCameraInfo

/// reads or sets default values for parameters
void ESMOCV::fillDefaults() {
   
  imageWidth = 1280; // flea3 default size
  imageHeight = 960; // flea3 default size
  //////// tracking parameters ///////////
  nh_.param("track/maxIte",p.maxIte,30);
  nh_.param("track/mprec",tparam.mprec,5);
  nh_.param("track/correlation_window",tparam.correlation_window,10);
  nh_.param("track/max_RMS",tparam.max_RMS,50.0);
  //////// visualization window parameters
  nh_.param("viewscale",viewscale,0.5);
  nh_.param("ploteveryframe",plot_every_frame,5);
  nh_.param("stepmask/step",stepmask_,1);
  
  
}//fillDefaults

/**
 * \brief reads the ROI parameters and return a rect with it.
 * \par use setROI with this rect to actually set the ROI. it is not just setting a rect.
 * \par supposes that imageHeight and imageWidth are correctly set!!
 * \par if roi/width and roi/height do not exist, default is 200x200
 * \par if roi/x or roi/y do not exist:
 * \li the ROI is centered on the middle of the image_height
 * \li x and y are appropriatelly set. They must exist.
 */
cv::Rect ESMOCV::readROIParams() {
  int x; int y; int roiw,roih;
  
  nh_.param("roi/width", roiw ,200);
  nh_.param("roi/height", roih ,200);
  if (nh_.hasParam("roi/x"))
    nh_.param("roi/x",x,(imageWidth/2)-(roiw/2));
  else { // the control needs roi/x and roi/y params.
    x = (imageWidth/2)-(roiw/2);
    nh_.setParam("roi/x",x);
  }
  if (nh_.hasParam("roi/y"))
    nh_.param("roi/y",y, (imageHeight/2)-(roih/2) );
  else { // the control needs roi/x and roi/y params.
    y = (imageHeight/2)-(roih/2);
    nh_.setParam("roi/y",y);
  }
  return Rect(x,y,roiw,roih);
}//readROIParams
 
/// some variables for dvs need the ROI. So here the ROI is set and we calculate these variables
void ESMOCV::setROI(Rect &newR){
  
  ROI = newR;
  //G0 = [1,0,winPos(1); 0,1,winPos(2); 0,0,1];
  G0 = (Mat_<double>(3,3) <<1, 0, ROI.x, 
		  0, 1, ROI.y,
		  0, 0, 1)  ;
  //recDim = [1,winPos(3),winPos(3),1,1; 1,1,winPos(4),winPos(4),1; ones(1,5)];
  recDim = (Mat_<double>(3,5) << 1, ROI.width, ROI.width, 1, 1,
	   1, 1, ROI.height, ROI.height,1,
	   1,1,1,1,1);
  //recCur = G0*recDim; recCur = recCur./repmat(recCur(3,:),3,1);
  recCur = G0 * recDim;
  dvsutil::Ut::unhomo(recCur);
  
  G = G0.clone();
  
  //pt = [winPos(1)+round(winPos(3)/2); winPos(2)+round(winPos(4)/2); 1];
  pt = (Mat_<double>(3,1) << ROI.x + round(ROI.width/2.0),
			     ROI.y + round(ROI.height/2.0),
			      1);
  
		  
}//setRect


/**
 * \brief sets the last parameters and then sets the reference image. Also draws the reference image with its ROI target
 * \par it must be called before tracking can be performed, with the first image
 * \par we use the first image to define image size (width x height) 
 * \par performs deep copy of the reference image to store it.
 */
void ESMOCV::setIref(const Mat &I) {
  Iref = I.clone(); // can not be shared copy, as there it is the same buffer for each image
  
  // uses the first image to define image size
  imageHeight = I.rows;
  imageWidth = I.cols;
  nh_.setParam("image_height", imageHeight);
  nh_.setParam("image_width", imageWidth);
  
  // with the image size, read the roi/ parameters and set the ROI.
  cv::Rect R = readROIParams(); 
  setROI(R);
  //patr = Ir(winPos(2):winPos(2)+winPos(4)-1, winPos(1):winPos(1)+winPos(3)-1);
  patr = Mat(Iref, ROI); // SHARED COPY!!!
  
  fillBogusCameraInfo(); // recalculate K which depends on image size
  
  //////////// build tparam for ESMTrack //////////////
  tparam.miter = p.maxIte;
  tparam.height = imageHeight;
  tparam.width = imageWidth;
  tparam.posx=ROI.x;
  tparam.posy=ROI.y;
  tparam.sizx=ROI.width;
  tparam.sizy=ROI.height;
  track.setParam(tparam);
  
  if (stepmask_ > 0) {
    // create mask
    StepMaskCreator mc(stepmask_);
    cv::Mat mask =  mc.createMask(ROI);
    //cv::imshow("debug mask esmocv.cpp", mask);
    track.setReferenceImageWithMask(I,mask,false); // REFERENCE ON TRACK
  } else 
    track.setReferenceImage(I,false); // REFERENCE ON TRACK
  
  //////////// draws image ref with ROI ////////////
  cv::rectangle( Iref,Point(tparam.posx,tparam.posy),Point(tparam.posx+tparam.sizx,tparam.posy+tparam.sizy),Scalar( 255, 255, 255 ));
  cv::imshow(REFWINDOW, Iref);
  
  hasIref=true; // habemus Iref!!!
}//setIref

/**
 * \brief higher level function, performs the tracking and draws the result
 * \par supposes everything is setup, if it has not a reference, it will call setIref and will not touch on H
 * \par because of cv::waitkey, needed to handle windows, we wait here for 1ms
 */
void ESMOCV::procImg(const Mat &I, Mat &H_,nodelet::ESMTrack &trackmsg) {
  
  if (!hasIref) {
      setIref(I);
  } else { // here is the main show.
    frameCount++;
    
   
    ///////////////// TRACKING /////////////
    if (track.track(I,G,trackmsg,false)) {// the boolean arg is supposed to ask it to save images, but it is not working
      stop_ = true; // with error on tracking we will stop the robot.
    }//if track  
    H_=G;
    recCur = G * recDim;
    dvsutil::Ut::unhomo(recCur);
    //Gc = G*inv(G0);
    //ptw(:,iter) = Gc*pt; ptw(:,iter) = ptw(:,iter)./repmat(ptw(3,iter),3,1);
    Gc = G * (G0.inv());
    // calculate the control point in the tracked image and save it into trackms
    ptw = Gc * pt; dvsutil::Ut::unhomo(ptw);
    trackmsg.ptw.x = ptw.at<double>(0,0);
    trackmsg.ptw.y = ptw.at<double>(1,0);
    trackmsg.ptw.theta  = 0;
    
        
    /////////// DRAW IMAGE WITH RESULT /////////////////
    if ((frameCount % plot_every_frame)==0) { // do not draw everyframe
      // we need another image to be able to draw over it, we use Ismall
      resize(I,Ismall,Size(),viewscale,viewscale,INTER_NEAREST);
      DrawResult(Ismall,viewscale);
      // the circle is the control point
      cv::circle(Ismall, cv::Point(trackmsg.ptw.x*viewscale,trackmsg.ptw.y*viewscale),
		 3, cv::Scalar(255,0,0), 2);
      cv::imshow(WINDOW, Ismall);
      // here it waits for 1ms, but we can not wait less time.
      cv::waitKey(1); // needed for imshow http://stackoverflow.com/questions/5217519/opencv-cvwaitkey
    
      // tried updateWindow instead of waitKey, but got the error below:
      //OpenCV Error: No OpenGL support (The library is compiled without OpenGL support)
      cv::updateWindow(WINDOW);
      
    }////////// endif DRAW IMAGE WITH RESULT /////////////////
    
  }//else hasIref
}//procImage



   /**
    * \brief draws G*ROI over the image, scaled with scale.
    * \par in other words, draws the tracked rectangle target over the image.
    * \par it is done by hand because it was copied from ESMlib. 
    * \param I as this image is modified, it can not be const
    */
void ESMOCV::DrawResult (Mat &I, double scale) 
  {
  int i;
  float pEnd[8], pIni[8];

  // for points in the corners of the target rectangle
  pIni[0] = 0;    pIni[1] = 0;      pIni[2] = tparam.sizx - 1; pIni[3] = 0;
  pIni[4] = tparam.sizx-1; pIni[5] = tparam.sizy - 1; pIni[6] = 0;      pIni[7] = tparam.sizy - 1;
  // perform G * those points (the points are unhomogeneous)
  for (i = 0; i < 4; i++) {
    pEnd[2*i] = (G.at<double>(0,0)*pIni[2*i] + G.at<double>(0,1)*pIni[2*i+1] + G.at<double>(0,2))/
      (G.at<double>(2,0)*pIni[2*i] + G.at<double>(2,1)*pIni[2*i+1] + G.at<double>(2,2)); 
    pEnd[2*i+1] = (G.at<double>(1,0)*pIni[2*i] + G.at<double>(1,1)*pIni[2*i+1] + G.at<double>(1,2))/
      (G.at<double>(2,0)*pIni[2*i] + G.at<double>(2,1)*pIni[2*i+1] + G.at<double>(2,2));      
  }
  // here the Point fomat is x,y
  DrawLine( I, Point((int)(pEnd[0]*scale),(int)(pEnd[1]*scale)), Point((int)(pEnd[2]*scale), (int)(pEnd[3]*scale)) );
  DrawLine( I, Point((int)(pEnd[2]*scale),(int)(pEnd[3]*scale)), Point((int)(pEnd[4]*scale), (int)(pEnd[5]*scale)) );
  DrawLine( I, Point((int)(pEnd[4]*scale),(int)(pEnd[5]*scale)), Point((int)(pEnd[6]*scale), (int)(pEnd[7]*scale)) );
  DrawLine( I, Point((int)(pEnd[6]*scale),(int)(pEnd[7]*scale)), Point((int)(pEnd[0]*scale), (int)(pEnd[1]*scale)) );
  
  // here the parameters are row, collumn
  //DrawLine (&I, (int)pEnd[1], (int)pEnd[0], (int)pEnd[3], (int)pEnd[2]);
  //DrawLine (&I, (int)pEnd[3], (int)pEnd[2], (int)pEnd[5], (int)pEnd[4]);
  //DrawLine (&I, (int)pEnd[5], (int)pEnd[4], (int)pEnd[7], (int)pEnd[6]);
  //DrawLine (&I, (int)pEnd[7], (int)pEnd[6], (int)pEnd[1], (int)pEnd[0]);

  return;
  }//DrawResult
  
  /// just encapsulates cv::line to keep consistent line drawing parameters
  void ESMOCV::DrawLine( Mat &img, Point start, Point end )
{
  int thickness = 2;
  int lineType = 8;
  //check if it will not segfault by drawing outside of range. (athough the docs say it is clipped)
  if ((start.x >= 0) && (start.y >= 0) && (end.x >= 0) && (end.x >= 0) && // check limits
      (start.x < img.cols) && (start.y < img.rows) && (end.x < img.cols) && (end.y < img.rows) &&
      ( start != end )  // check if the points are different
     ) {
    cv::line( img,
        start,
        end,
        Scalar( 255, 255, 255 ),
        thickness,
        lineType );
  } else {
    
  }
}//DrawLine


Mat ESMOCV::getSmallImage() {
    return Ismall;
}

} //namespace
  