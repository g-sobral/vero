
#include "DvsCtrl.h"


using namespace cv;

namespace dvs
{
            
         
int DvsCtrl::doCtrl(const Mat &G, const Mat &K, const Mat &pt,esmocv::Dvs_Status &dvsstat) {

/* original matlab code
H = inv(K) * G * K;
m = inv(K) * pt;
[mu,vartheta] = axe_ang(H);
e(1:3,1) = (H - eye(3))*m;
e(4:6,1) = vex(H - H');
%e(4:6,1) = vartheta*mu;
v_c = lambda * e;
*/
//e = Mat::zeros(6,1,CV_64F);

e_ = Mat::zeros(6, 1, CV_64F); // init e
Mat v_c = Mat::zeros(6, 1, CV_64F); // init v_c
Mat I = Mat::eye(3, 3, CV_64F);
Mat H = K.inv() * G * K;
Mat m = K.inv() * pt;
Mat mu; double vartheta;
axe_ang( H, mu, vartheta);

Mat e1 = (H - I) * m;
e1.copyTo(e_(cv::Rect(0,0,1,3)));// rect(x,y,width,height of e <- e2
Mat e2 = dvsutil::Ut::vex(H-H.t());
e2.copyTo(e_(cv::Rect(0,3,1,3)));// rect(x,y,width,height of e <- e2

v_c = adapt_gain() * e_;

dvsutil::Ut::copyMat2Array(e_, dvsstat.e);
dvsutil::Ut::copyMat2Array(v_c, dvsstat.v_c);

ROS_INFO_THROTTLE(10,"DVS ERR %.3g %s %.3f"
		      ,norm(e_)
		      ,((param.adapt_gamma > 0)?"ADPT":"gain")
		      ,adapt_gain());


/* // print outs for debug
std::stringstream ss;
ss << "\n m " << m;
ss << "\n e " << e;
ss << "\n e2 " << e2;
ss << "\n K " << K;
ss << "\n H " << H;
ss << "\n G " << G;
ss << "\n mu " << mu;
ss << "\n vartheta " << vartheta;
ss << "\n DvsCtrl: lamda " << param.lambda ;
ROS_INFO_THROTTLE(2,ss.str().c_str());*/
return 0;
}//doCtrl

} // namespace