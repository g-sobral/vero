
#include "DDvsCtrl.h"


using namespace cv;
using namespace dvsutil;

namespace dvs
{
            
         
int DDvsCtrl::doCtrl(const Mat &G, const Mat &K, const Mat &pt, esmocv::Dvs_Status &dvsstat) {

/* original matlab code
H = inv(K) * G * K;
m = inv(K) * pt;

e(1:3,1) = (H - eye(3))*m;
e(4:6,1) = vex(H - H');

%M = [2*eye(3,3) ppv(m); zeros(3,3) eye(3,3)];
M = [2*eye(3,3) ppv(m); -ppv(m) eye(3,3)];

v_c = lambda * M * e;
*/
e_ = Mat::zeros(6, 1, CV_64F); // init e
Mat v_c = Mat::zeros(6, 1, CV_64F); // init v_c
Mat I = Mat::eye(3, 3, CV_64F);
Mat H = K.inv() * G * K;
Mat m = K.inv() * pt;
Mat e1 = (H - I) * m;
e1.copyTo(e_(cv::Rect(0,0,1,3)));// rect(x,y,width,height of e <- e2
Mat e2 = Ut::vex(H - H.t());
e2.copyTo(e_(cv::Rect(0,3,1,3)));// rect(x,y,width,height of e <- e2
Mat M =  Mat::zeros(6, 6, CV_64F);
Mat I2 = 2*I;
Mat ppvm = Ut::ppv(m);
I2.copyTo(M(cv::Rect(0,0,3,3)));
ppvm.copyTo(M(cv::Rect(3,0,3,3)));

/// not using ppv correspond to this variant %M = [2*eye(3,3) ppv(m); zeros(3,3) eye(3,3)];
if (useppv_) {
  // and using ppv is this one: M = [2*eye(3,3) ppv(m); -ppv(m) eye(3,3)];
  ppvm = -1 * ppvm; // negates ppvm
  ppvm.copyTo(M(cv::Rect(0,3,3,3)));
}

I.copyTo(M(cv::Rect(3,3,3,3)));

v_c = adapt_gain() * M * e_;

/// write results in the message to be transmitted
dvsutil::Ut::copyMat2Array(e_, dvsstat.e);
dvsutil::Ut::copyMat2Array(v_c, dvsstat.v_c);

ROS_INFO_THROTTLE(10,"DDVS ERR %.3g %s %.3f, M mtx w/ %s",norm(e_)
		      ,((param.adapt_gamma > 0)?"ADPT":"gain")
		      ,adapt_gain()
		      ,(useppv_?"-ppv":"zeros"));
/*
// print outs for debug
std::stringstream ss;
ss << "\n m " << m;
ss << "\n e " << e;
ss << "\n e2 " << e2;
ss << "\n K " << K;
ss << "\n H " << H;
ss << "\n G " << G;
ss << "\n -ppvm " << ppvm;
ss << "\n M " << M;
ss << "\n DDvsCtrl: lamda " << param.lambda ;
ROS_INFO_THROTTLE(2,ss.str().c_str());
*/
return 0;
}//doCtrl

} // namespace