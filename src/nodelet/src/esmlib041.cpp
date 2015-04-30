
#include "esmlib041.h"

namespace esmlib
{
   int ESMlibTrack::setReferenceImageWithMask( const cv::Mat &image_ref, const cv::Mat &mask, bool saveIref=false) {
     #if defined(__i386__) || defined(__ILP32__) 
     perror("ESMlib MASK can only be used on 64 bit architecture!");
     exit(0);
     #endif
     mask.convertTo(mask_,CV_32F); // ALWAYS DEEP COPY TO KEEP the MASK     
     // save image_ref into I (type conversion)
     Imask.data = (float*)mask_.data;
     Imask.cols = tparam_.sizx;
     Imask.rows = tparam_.sizy;
     
     use_mask_ = true;
     return setReferenceImage(image_ref,saveIref);
   }//setReferenceImageWithMask
  
  
   int ESMlibTrack::setReferenceImage( const cv::Mat &image_ref,  bool saveIref=false) {
     H_ = cv::Mat::eye(3,3,CV_32F);
     // convert image_ref into a float image
     image_ref.convertTo(imgRef_,CV_32F); // ALWAYS DEEP COPY TO KEEP REF IMAGE
     // save image_ref into I (type conversion)
     I.data = (float*)imgRef_.data;
     I.cols = tparam_.width;
     I.rows = tparam_.height;
     int ret_MallTrack;
     if (use_mask_) {
       #if defined(__LP64__)
        ROS_INFO_STREAM("ESMTrack64b init (mask): " << tparam_);
	ret_MallTrack = MallTrackMask (&T, &I, &Imask, tparam_.posx, tparam_.posy, tparam_.sizx, tparam_.sizy, tparam_.miter, tparam_.mprec);
        showIstructMask(GetPatm(&T), "mask pattern");
	
       #endif
     } else {
       ROS_INFO_STREAM("ESMTrack init (no mask): " << tparam_);
       ret_MallTrack = MallTrack (&T, &I, tparam_.posx, tparam_.posy, tparam_.sizx,
		    tparam_.sizy, tparam_.miter, tparam_.mprec);
     }
       
     if (ret_MallTrack) {
       ROS_FATAL_STREAM("ESMLibTrack::setROI() ESMlib alloc ERR "<< ret_MallTrack << " w/ params: " << tparam_ );
       return (1);
     } else 
      ROS_INFO("ESM Tracking structure allocated");
     hasref=true;
     //tparam_.print(); // debug
     
     if (saveIref) {
       ROS_INFO("ESM Tracking: Saving Reference Image");
       SavePgm((char*)"./Iref.pgm",&I);
       SavePgm((char*)"./patref.pgm", GetPatr (&T));
     }
     
     return 0;
   }//setReferenceImage

   /**
    * \brief actually performs ROI tracking
    * @param image if it is not FLOAT (32bit), it will be converted to float (slower). Because ESMlib needs float images.
    * @param saveImg for debugging purposes, it is supposed to save images. But it appears not to be working.
    */
   int ESMlibTrack::track( const cv::Mat &image , cv::Mat &H, nodelet::ESMTrack &trackstats, bool saveImg=false) {
     if (!hasref) return 1;
     
    fpsutil_.cycleHZ(trackstats_.fps); // write fps in the message
     
     bool converted2float = false;
     if (image.depth()!=CV_32F) {  
       // convert image_ref into a float image (DEEP COPY)
       image.convertTo(img_,CV_32F);
       // save image_ref into I (type conversion)
       I.data = (float*)img_.data;
       converted2float = true;
     } else { // image is already float. do not need to deep copy or convert
       I.data = (float*)image.data;
     }
 
     timeutil_.cycle(); // start counting time...
     
     if (tparam_.correlation_window) {
      if (converted2float)
	correlate(img_,false);
      else
	correlate(image,false);
      
      timeutil_.cycle(trackstats_.t_corr); // gets time to perform correlation
      
      // the clone time is not used anymore. we are not cloning
      timeutil_.cycle(trackstats_.t_clones); // gets time to perform cloning
    }
    
     int ret_maketrack; // stores the return value of the tracking function
     if (use_mask_) {
       #if defined(__LP64__)
       ret_maketrack = MakeTrackMask (&T, &I);
       #endif
     } else
       ret_maketrack = MakeTrack (&T, &I);
     
     if (ret_maketrack)
      return (-1);
     trackstats_.header.stamp = ros::Time::now(); // save timestamp
     timeutil_.cycle(trackstats_.t_track); // get time to perform tracking
          
     if (saveImg) { // the SavePgm function appears not to be working
       ROS_INFO_THROTTLE(1,"ESM Tracking: Saving Image miter %d mprec %d",T.miter,T.mprec);
       DrawResult (tparam_.sizx, tparam_.sizy, T.homog, I); 
       char filename[50];
       sprintf(filename,"/tmp/res/I%03ld.pgm",fpsutil_.getCount());
       SavePgm(filename,&I);
       sprintf(filename,"/tmp/res/pat%03ld.pgm",fpsutil_.getCount());
       SavePgm (filename, GetPatr (&T));
     }
     
     /////// RETURNING VALUES //////////
     // manually copy T.homog into H. guarantee no alocation
     H_.at<float>(0,0) = T.homog[0];
     H_.at<float>(0,1) = T.homog[1];
     H_.at<float>(0,2) = T.homog[2];
     H_.at<float>(1,0) = T.homog[3];
     H_.at<float>(1,1) = T.homog[4];
     H_.at<float>(1,2) = T.homog[5];
     H_.at<float>(2,0) = T.homog[6];
     H_.at<float>(2,1) = T.homog[7];
     H_.at<float>(2,2) = T.homog[8];
     //also copy T.homoh into trackstats_
     for (int j = 0; j < 9 ; j++)
       trackstats_.H[j] = T.homog[j];
     
     H_.convertTo(H,CV_64F);/// deep copy, do not trust the user. ( returning H to the caller)
     /////// end RETURNING VALUES //////////
     
     /// ///////// safety (zncc and RMS)
     trackstats_.stop = 0; // default, we continue
     if (tparam_.max_RMS > 0) /// if max_RMS is not set, do not calculate it
      trackstats_.RMS = calcRMSerror();
     #if defined(__LP64__)
      trackstats_.zncc = GetZNCC(&T);
     #else
      trackstats_.zncc = 1.0;
     #endif
     if ((trackstats_.zncc < tparam_.zncc_min) &&
       (trackstats_.RMS > tparam_.max_RMS) ) { // if both are bad
	ROS_INFO_THROTTLE(2,"ESMTrack: BREAK zncc %.3f > %.3f zncc_min AND RMSerr %.3f > %.3f max_RMS "
			  ,trackstats_.zncc,tparam_.zncc_min,trackstats_.RMS,tparam_.max_RMS);
       trackstats_.stop = 1; // houston, we have a problem! can we stop?
     } else { // now check each one separately
      if (trackstats_.zncc < tparam_.zncc_min) {
	ROS_INFO_THROTTLE(2,"ESMTrack: BREAK zncc %.3f > %.3f zncc_min",trackstats_.zncc,tparam_.zncc_min);
	trackstats_.stop = 1; // houston, we have a problem! can we stop?
	}
      if (trackstats_.RMS > tparam_.max_RMS) {
	ROS_INFO_THROTTLE(2,"ESMTrack: BREAK RMSerr %.3f > %.3f max_RMS",trackstats_.RMS,tparam_.max_RMS);
	trackstats_.stop = 1; // houston, we have a problem! can we stop?
	}
     }//else
     /// //////////////// end safety
      
      /// debug stuff, lets see what are the returned images.
      // reference patter is static
      //showIstruct(GetPatr(&T), "reference pattern");
      // current patter is the current target area on image warped to appear immobile.
      //showIstruct(GetPatc(&T), "current pattern");
      // mask pattern is a black image, as we have not defined a pattern
      //showIstructMask(GetPatm(&T), "mask pattern");
      /// debug /////////
      
      
      
     trackstats = trackstats_; /// copy stats into output
     return trackstats_.stop;// everything is ok.
   } //Track
   
   int ESMlibTrack::init() {
     imgRef_ = cv::Mat::zeros(tparam_.width,tparam_.height,CV_32F); // pre-allocates
     img_ = cv::Mat::zeros(tparam_.width,tparam_.height,CV_32F); // pre-allocates
     fpsutil_.reset(); // start counting images again from zero.
     if (hasref) 
      if (!use_mask_) FreeTrack(&T);
      else { 
       #if defined(__LP64__)
	FreeTrackMask(&T);
       #elif defined(__LP64__)
	ROS_FATAL("ESMLibTrack::init ERROR Can not use mask on 32 bit arch.");
       #endif
     }
     use_mask_ = false;
     hasref=false;
     return 0;
   }//init
   

   /**
    * \brief performs correlation of the current and reference image, and updates H accordingly
    * \par if you do not know what you are doing, f**k off. It is not heavily commented just 'cause I enjoyed commenting. Turn the correlation off if you are not happy with it.
    */
  void ESMlibTrack::correlate(const cv::Mat &newimage, bool corr_debug) {
    if (fpsutil_.getCount() < 2) return;
    bool target_full_visible = false; /// is the target completelly visible in the image?
    
    ///variable to get a margin of some pixels around a rect. will be used later
    cv::Size s(2*tparam_.correlation_window,2*tparam_.correlation_window);
    cv::Point p(tparam_.correlation_window,tparam_.correlation_window);
    
    // init matriz with homogeneous ROI vertices, each collumn is a point
    cv::Mat pIni = (cv::Mat_<float>(3,4) << 
    0,	tparam_.sizx-1,	tparam_.sizx-1,	0,  
    0,	0,		tparam_.sizy-1,	tparam_.sizy-1,
    1,	1,		1,		1 
    );
    
    //std::cerr << "XXXXXXXXXXXXXXXXXXXXXXXXX";
    cv::Mat pWpd = H_ * pIni; // warp pIni by H to get the target ROI in the current image
    dvsutil::Ut::unhomo( pWpd ); 
    
    /// get a rect representing the size of the whole new image
    cv::Size isize = newimage.size();// get the size of the new image
    cv::Rect wholeimage(0,0,isize.width-1,isize.height-1);
    
    /// get a bounding rect around the warped ROI. this would be the correlation kernel
    cv::Rect boundingROI = dvsutil::Ut::boundingBoxOf2DPoints( pWpd ); 
    
    // if the intersection is the boundingROI, the target is fully visible.
    target_full_visible = (boundingROI == (boundingROI & wholeimage));
            
    cv::Rect searchROI = (boundingROI);// - p)+s;
    
    /// make sure searchROI does not go outside of new image
    // rect & rect means intersection of rectangles
    searchROI &= wholeimage;
    
    // another header gets the region to be searched. O(1), NO deep copy.
    cv::Mat Isearch = cv::Mat(newimage,searchROI); // NO deep copy
    
    ///////////// new coorrelation with warped image and refrence
    cv::Mat Iwarped;
    // translation from searchROI to current image.
    cv::Mat Tw = (cv::Mat_<float>(3,3) << 
    1 , 0 ,  searchROI.x,
    0 , 1 ,  searchROI.y,
    0 , 0 ,  1            );
        
    // ********************************************************
    // here it is tricky. read carrefully. if you don't know what you are doing, f**k off!!!
    // H goes from (0,0) on the corner of ROI, not of RefImg
    // THerefore, H.inv goes from current image to ROI. 
    // it is correct to use Tw to translate from searchROI to current image
    // in the result, what is above or left of the ROI will get negative coordinates, 
    // and will be discarded (implementation detail)
    // if we ask for the exact size of the ROI, 
    // everything outside of the ROI will go outside and also disappear.
    // so, this gets the ROI in the current image, warped back to match the original ROI.
    try {
    warpPerspective(Isearch, Iwarped, H_.inv()*Tw,cv::Size(tparam_.sizx,tparam_.sizy));
    } catch (cv::Exception e) {
      std::stringstream ss;
      ss << "ESMTrack::correlate has thrown " << e.err << " at function " << e.func;
      ROS_ERROR_THROTTLE(3,"%s",ss.str().c_str());
      return; // no sense to continue, correlation has failed. 
    }//catch
    
    if (corr_debug) cv::imshow("debug corr wpd",Iwarped*(1/255.0));
        
    /// get the ROI plus the correlation border on the reference image
    cv::Rect theROI(tparam_.posx,tparam_.posy,tparam_.sizx,tparam_.sizy);
    // rect + point means shifting the rect; rect + size means changing the size of rect
    cv::Rect theROIPlusCorr = (theROI - p)+s; // the roi with a marging around it.
    // if the ROI is too close of the image border, the correlation border may go out of the image
    // in this case, we must calculate how much, and then take the difference
    cv::Rect wholeREFimage(0,0,imgRef_.cols,imgRef_.rows);
    cv::Point ROIoutofBorder(0,0);
    if (theROIPlusCorr.x < 0)
      ROIoutofBorder.x = theROIPlusCorr.x;
    if (theROIPlusCorr.y < 0)
      ROIoutofBorder.y = theROIPlusCorr.y;
    // guarantee that the ROI does not go out of the image.
    theROIPlusCorr &= wholeREFimage;
    cv::Mat IrefPatr = cv::Mat(imgRef_,theROIPlusCorr); // **NO** deep copy here.
    
    if (corr_debug) {
      cv::imshow("debug patref",IrefPatr*(1/255.0));
    
      ////////////// debug image....
      cv::Mat Ic;
      newimage.convertTo(Ic,CV_8UC1); /// deep copy, but this is debug!!!! no production code!!
      for (int k=0; k<4; k++) {
	cv::circle(Ic, cv::Point((int)pWpd.at<float>(0,k),(int)pWpd.at<float>(1,k)), 5, cv::Scalar( 200, 200, 222 ),4);
	line(Ic, cv::Point((int)pWpd.at<float>(0,k),(int)pWpd.at<float>(1,k)),
	   cv::Point((int)pWpd.at<float>(0,(k+1)%4),(int)pWpd.at<float>(1,(k+1)%4)), 
	   cv::Scalar( 200, 200, 222 ),3);
      }//for
      cv::rectangle(Ic, boundingROI,cv::Scalar( 150, 150, 150 ),2,8);
      cv::rectangle(Ic, wholeimage,cv::Scalar( 255, 255, 255 ),2,8);
      //cv::rectangle(Ic, boundingROI+offset,cv::Scalar( 255, 255, 255 ),2,8);
      cv::rectangle(Ic, searchROI,cv::Scalar( 254, 254, 254 ),2,8);
      cv::namedWindow("debug2 corr searchROI"); // you can call it many times, if the window exists it does nothing.
      cv::imshow("debug2 corr searchROI",Ic);//.mul(1.0/255.0)); // DEEP COPY
      cv::waitKey(1);// here it looses 1 ms, but we need it.
      /////////////////
    }//corr_debug
    
    
    cv::Mat matchres;
    // the template can not be bigger than the image where we search for it.
    CV_Assert(Iwarped.cols <= IrefPatr.cols); 
    CV_Assert(Iwarped.rows <= IrefPatr.rows);
    // the show runs here.
    cv::matchTemplate(IrefPatr, Iwarped, matchres,CV_TM_CCORR_NORMED);
    double min_val,max_val;cv::Point minloc,maxloc;
    cv::minMaxLoc(matchres, &min_val, &max_val, &minloc, &maxloc);
    
    // the correlation is inverted, so the - 
    cv::Point offset = -(maxloc - cv::Point(tparam_.correlation_window-ROIoutofBorder.x
					 ,tparam_.correlation_window-ROIoutofBorder.y));
    if (corr_debug) {
      std::cerr << "\n ROIoutofBorder " << ROIoutofBorder;
      std::cerr << "\n MAXLOC " << maxloc;
      std::cerr << "\n offset " << offset;
    } //if corr_debug
    
    //////copy the offset into trackstats_
    trackstats_.corr_offset.x = offset.x;
    trackstats_.corr_offset.y = offset.y;
    trackstats_.corr_offset.theta = 0;
    
    /// the offset in the form of a translation matrix
    cv::Mat  offsetmtx = (cv::Mat_<float>(3,3) <<1, 0, offset.x, 
		                               0, 1,   offset.y,
		                               0, 0,          1)  ;
    /// apply the translation on the previous homography
    H_ = offsetmtx * H_;
  
					       
    //ROS_INFO_THROTTLE(2,"H bef corr %.3f %.3f %.3f",T.homog[2],T.homog[5],T.homog[8]);
    /// update homography on T... we hope it gets more robust to movement.
    // yeah, I checked, it really matters if you change T.homog.
    T.homog[0] = H_.at<float>(0,0);
    T.homog[1] = H_.at<float>(0,1);
    T.homog[2] = H_.at<float>(0,2);
    T.homog[3] = H_.at<float>(1,0);
    T.homog[4] = H_.at<float>(1,1);
    T.homog[5] = H_.at<float>(1,2);
    T.homog[6] = H_.at<float>(2,0);
    T.homog[7] = H_.at<float>(2,1);
    T.homog[8] = H_.at<float>(2,2);
    //ROS_INFO_THROTTLE(2,"H aft corr %.3f %.3f %.3f",T.homog[2],T.homog[5],T.homog[8]);
    //std::cerr << "\ng";
    
    if (corr_debug) {
      /// debug window (correlation result)
      cv::namedWindow("debug correlation result"); // you can call it many times, if the window exists it does nothing.
      cv::circle(matchres, cv::Point(maxloc.x,maxloc.y),2, cv::Scalar(150,150,150), 2);
      cv::rectangle(matchres, cv::Rect(0,0,matchres.cols-1,matchres.rows-1),cv::Scalar( 150, 150, 150 ),2,8);
      cv::imshow("debug correlation result",(matchres-min_val)/(max_val-min_val)); // DEEP COPY
    } // if (corr_debug) 
    
    
  }//Correlate
   
   
  
void ESMlibTrack::DrawLine (imageStruct *image, int r1, int c1, int r2, int c2)
{
  int dr, dc, temp;
  int cols = image->cols, rows = image->rows;
  int i, point, area;
  
  area = cols * rows;
  if (r1 == r2 && c1 == c2) 
    return;
  if (abs (r2 - r1) < abs (c2 - c1)) {
    if (c1 > c2) {
      temp = r1; r1 = r2; r2 = temp;
      temp = c1; c1 = c2; c2 = temp;
    }
    dr = r2 - r1;
    dc = c2 - c1;
    temp = (r1 - c1 * dr / dc)*cols;
    for (i = c1; i <= c2; i++) {
      point = temp + (i * dr) / dc * cols  + i;
      if ((point >= 0) & (point < area))  
        image->data[point] = 0.0;
    }
  } 
  else {
    if (r1 > r2) {
      temp = r1; r1 = r2; r2 = temp;
      temp = c1; c1 = c2; c2 = temp;
    }
    dr = r2 - r1;
    dc = c2 - c1;
    temp =  c1 - r1 * dc / dr;
    for (i = r1; i <= r2; i++) {
      point = temp + i*cols + (i * dc) / dr;
      if ((point >= 0) & (point < area))  
        image->data[point] = 0.0;
    }
  }

  return;
  }
   
  void ESMlibTrack::DrawResult (int sx, int sy, float H[9], imageStruct I) 
  {
  int i;
  float pEnd[8], pIni[8];

  pIni[0] = 0;    pIni[1] = 0;      pIni[2] = sx - 1; pIni[3] = 0;
  pIni[4] = sx-1; pIni[5] = sy - 1; pIni[6] = 0;      pIni[7] = sy - 1;
  for (i = 0; i < 4; i++) {
    pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]); 
    pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
      (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);      
  }
  DrawLine (&I, (int)pEnd[1], (int)pEnd[0], (int)pEnd[3], (int)pEnd[2]);
  DrawLine (&I, (int)pEnd[3], (int)pEnd[2], (int)pEnd[5], (int)pEnd[4]);
  DrawLine (&I, (int)pEnd[5], (int)pEnd[4], (int)pEnd[7], (int)pEnd[6]);
  DrawLine (&I, (int)pEnd[7], (int)pEnd[6], (int)pEnd[1], (int)pEnd[0]);

  return;
  }//DrawResult

  /**
   * \brief shows whatever is in a imagestruct, in a ocv window named name
   * \par DEEP COPY - FOR DEBUG. DO NOT USE ON LOOPS ON PRODUCTION CODE
   * \par looses 1ms on waitKey - again, DEBUG ONLY
   */
  void ESMlibTrack::showIstruct(imageStruct *im, std::string name) {
    
    // allocate a cv::Mat header, with no data copying
    // from docs.opencv Mat img(height, width, CV_8UC3, pixels, step);
    // step is row length in bytes. if missing, assumes cols*size(element)
    cv::Mat I = cv::Mat(GetImageRows(im),GetImageCols(im),CV_32F,GetImageData(im)); 
    cv::namedWindow(name.c_str()); // you can call it many times, if the window exists it does nothing.
    // opencv draws float images with values from 0 to 1, it multiplies by 255 before drawing
    // but the esmlib already has values from 0 to 255, so we must divide
    // and we need to clone because you dont want to mess with the data on the T struct, do you?
    cv::imshow(name.c_str(),I.clone().mul(1.0/255.0)); // DEEP COPY
    cv::waitKey(1);// here it looses 1 ms, but we need it.
    
  }//ShowIstruct
   
   /**
   * \brief shows whatever is in a imagestruct with only 0 and 1 values, in a ocv window named name
   * \par NO DEEP COPY NEED because opencv already handles binary images
   * \par looses 1ms on waitKey - again, DEBUG ONLY
   */
  void ESMlibTrack::showIstructMask(imageStruct *im, std::string name) {
    
    // allocate a cv::Mat header, with no data copying
    // from docs.opencv Mat img(height, width, CV_8UC3, pixels, step);
    // step is row length in bytes. if missing, assumes cols*size(element)
    cv::Mat I = cv::Mat(GetImageRows(im),GetImageCols(im),CV_32F,GetImageData(im)); 
    cv::namedWindow(name.c_str()); // you can call it many times, if the window exists it does nothing.
    // opencv draws float images with values from 0 to 1, it multiplies by 255 before drawing
    // but the esmlib already has values from 0 to 255, so we must divide
    // and we need to clone because you dont want to mess with the data on the T struct, do you?
    cv::imshow(name.c_str(),I); 
    cv::waitKey(1);// here it looses 1 ms, but we need it.
    
  }//ShowIstruct
   
   /**
    * \brief computes RMS error. 
    * \par normally disabled as it takes noticiable time. Maybe it is worthy if implemented on GPU.
    */
  float ESMlibTrack::calcRMSerror() {
    
    /// get reference image
    imageStruct* ref = GetPatr(&T);
    // allocate a cv::Mat header, with no data copying
    // from docs.opencv Mat img(height, width, CV_8UC3, pixels, step);
    // step is row length in bytes. if missing, assumes cols*size(element)
    cv::Mat Iref = cv::Mat(GetImageRows(ref),GetImageCols(ref),CV_32F,GetImageData(ref)); 
    
    /// get current pattern 
    imageStruct* cur = GetPatc(&T);
    // allocate a cv::Mat header, with no data copying
    // from docs.opencv Mat img(height, width, CV_8UC3, pixels, step);
    // step is row length in bytes. if missing, assumes cols*size(element)
    cv::Mat Icur = cv::Mat(GetImageRows(cur),GetImageCols(cur),CV_32F,GetImageData(cur)); 
    
    cv::Mat Ierr = Icur - Iref;
    Ierr = Ierr.mul(Ierr);
    
  // compute sum of matrix elements, iterator-based variant
  // there are gpu functions in opencv to do that.
  float sum=0;
  cv::MatConstIterator_<float> it = Ierr.begin<float>(), it_end = Ierr.end<float>();
  for(; it != it_end; ++it)
    sum += *it;
  // sum has sum of matrix elements  
  return sqrt(sum/Ierr.total()); // total is number of elements in the matrix
    
  }//calcRMSerror
   
std::ostream& operator<<(std::ostream& out, const struct ESMTrackParam& p) {
  out << "ESMTrackParam: iter " << p.miter << " mprec " << p.mprec << " pos [" << p.posx <<
   "," << p.posy << "] siz ["<< p.sizx <<","<<p.sizy<<"] wid " << p.width << " hei " << p.height << " zncc " << p.zncc_min << " RMSmx " << " corr "<< p.correlation_window << p.max_RMS;
}
   
   
}//namespace


