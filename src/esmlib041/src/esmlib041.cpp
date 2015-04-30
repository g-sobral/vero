
#include "esmlib041.h"

namespace esmlib
{

   int ESMlibTrack::setReferenceImage( const cv::Mat &image_ref , bool saveIref=false) {
     
     // convert image_ref into a float image
     image_ref.convertTo(imgRef_,CV_32F); // ALWAYS DEEP COPY TO KEEP REF IMAGE
     // save image_ref into I (type conversion)
     I.data = (float*)imgRef_.data;
     I.cols = tparam_.width;
     I.rows = tparam_.height;
     if (MallTrack (&T, &I, tparam_.posx, tparam_.posy, tparam_.sizx,
		    tparam_.sizy, tparam_.miter, tparam_.mprec)) {
       ROS_FATAL("ESM allocation error ");
       return (1);
     } else 
      ROS_INFO("ESM Tracking structure allocated");
     hasref=true;
     countimg_=0;
     if (saveIref) {
       SavePgm((char*)"./res/Iref.pgm",&I);
       SavePgm((char*)"./res/patref.pgm", GetPatr (&T));
     }
     
     return 0;
   }//setReferenceImage

   int ESMlibTrack::track( const cv::Mat &image , cv::Mat &H, bool saveImg=false) {
     if (!hasref) return 1;
     if (image.depth()!=CV_32F) {
       // convert image_ref into a float image (DEEP COPY)
       image.convertTo(img_,CV_32F);
       // save image_ref into I (type conversion)
       I.data = (float*)img_.data;
     } else { // image is already float. do not need to deep copy or convert
       I.data = (float*)image.data;
     }
     if (MakeTrack (&T, &I))
      return (1);
     countimg_++;
     if (saveImg) {
       DrawResult (tparam_.sizx, tparam_.sizy, T.homog, I); 
       char filename[50];
       sprintf(filename,"./res/I%03d.pgm",countimg_);
       SavePgm(filename,&I);
       sprintf(filename,"./res/pat%03d.pgm",countimg_);
       SavePgm (filename, GetPatr (&T));
     }
     // manually copy T.homog into H. guarantee no alocation
     H.at<double>(0,0) = T.homog[0];
     H.at<double>(0,1) = T.homog[1];
     H.at<double>(0,2) = T.homog[2];
     H.at<double>(1,0) = T.homog[3];
     H.at<double>(1,1) = T.homog[4];
     H.at<double>(1,2) = T.homog[5];
     H.at<double>(2,0) = T.homog[6];
     H.at<double>(2,1) = T.homog[7];
     H.at<double>(2,2) = T.homog[8];
     
     return 0;
   } //Track
   
   int ESMlibTrack::init() {
     imgRef_ = cv::Mat::zeros(tparam_.width,tparam_.height,CV_32F); // pre-allocates
     img_ = cv::Mat::zeros(tparam_.width,tparam_.height,CV_32F); // pre-allocates
     countimg_=0;
     if (hasref) FreeTrack(&T);
     hasref=false;
     return 0;
   }//init
   
   
  
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
  }

   
   
}//namespace


