#include "Ctrl.h"

using namespace cv;
using namespace dvsutil;

namespace dvs
{
/**
 * \brief Determine the unit axis
 * \TODO move to dvsutil
 */
void Ctrl::axe_ang(const Mat &H, Mat &axe_rot, double &angle_rot) {

/* original matlab code
function [axe_rot,angle_rot] = axe_ang(H)
% Determine the unit axis
axe_rot = 0.5 * vex(H - H');
norm_axe_rot = norm(axe_rot);
if (norm_axe_rot>0.0000000001)
  axe_rot = axe_rot/norm_axe_rot;
else
  axe_rot = [0 0 1]';
end
% Determine the angle and its quadrant
if (trace(H)>=1)
  angle_rot = real(asin(norm_axe_rot));
else
  angle_rot = pi - real(asin(norm_axe_rot));
end
*/  
// Determine the unit axis
axe_rot = 0.5 * Ut::vex(H-H.t());
cv::Vec<double,3> vaxe_rot(axe_rot);
double norm_axe_rot = norm(vaxe_rot);
if (norm_axe_rot > 1e-10)
  axe_rot = axe_rot / norm_axe_rot;
else
  axe_rot = (Mat_<double>(3,1) << 0, 0, 1);
// Determine the angle and its quadrant
if (trace(H)[0] >= 1.0)
  angle_rot = asin(norm_axe_rot);
else 
  angle_rot = M_PI * asin(norm_axe_rot);

}// axe_ang

/**
 * \brief read parameters for ctrl algorithm
 * @param lambda (gain multiplier)
 * @param adapt_gamma if > 0, adaptive gain will be used and this is the exp decay parameter
 * @param stor_norm_error threshold on norm error value to stop the robot
 * @param stop_is_definitive if stop_norm_error is reached, the robot may stop definitively or only while the norm is lower than the threshold
 */
int Ctrl::readParam() {
  nh_.param("lambda", param.lambda,0.3);
  ROS_ASSERT_MSG( (param.lambda > 0),"the lambda control gain must be positive.");
  nh_.param("adapt_gamma", param.adapt_gamma,0.0);
  ROS_ASSERT_MSG( (param.adapt_gamma >= 0),"the gamma parameter (adaptive gain exp decay) must be positive or zero.");
  nh_.param("stop_norm_error", param.eps,1e-3);
  ROS_ASSERT_MSG( (param.eps > 0),"the error norm limit must be positive to define a stop condition");
  nh_.param("stop_is_definitive", param.stop_is_definitive,true);
}//readParam

/**
 * \brief calculate adaptive gain = lambda * exp ( - gamma norm(error) )
 * \par if gamma <= 0, the gain will be constant and equal to lambda
 */
double Ctrl::adapt_gain() {
  if (param.adapt_gamma <= 0) return param.lambda;  // constant gain
  return param.lambda * exp( - param.adapt_gamma * cv::norm(e_) ); // adaptive gain
}

std::ostream& operator<<(std::ostream &out, const struct CtrlParam &c){
    out << "CtrlParam lambda: " << c.lambda << " gamma: " << c.adapt_gamma << " eps: " << c.eps << " stop_definitive " << (c.stop_is_definitive?"T":"F") << " ";
    return out;
}


} // namespace