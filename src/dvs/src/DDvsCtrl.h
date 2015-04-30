
#ifndef DVS_DDVS_CTRL_H
#define DVS_DDVS_CTRL_H

#include "Ctrl.h"

using namespace cv;

namespace dvs
{
  
/**
 * \brief PURPOSE : direct visual servoing w.r.t. a planar object
 *
 * \par INPUT  
 * \li - G in SL(3) is the projective homography from the reference frame to the camera frame
 * \li - K in R^{3x3} is an estimate of the camera intrinsic params
 * \li - lambda > 0 is the control gain
 * \li pt = [u,v,1]^T in P^2 is the control point
 *
 * \par OUPUT   : v_c in R^6 contains the camera velocities
 *
 * \par AUTHOR  :
 * \li Geraldo Silveira
 * \li http://sites.google.com/site/geraldofsilveira
 * \li Geraldo.Silveira@cti.gov.br
 *
 * \par REFERENCE : 
 * G. Silveira and E. Malis, "Direct Visual Servoing 
 * with respect to Rigid Objects," Proc. of IEEE/RSJ IROS,
 * USA, pp. 1963-1968, October 2007. 
 *
 * \par CTI Renato Archer, August 2010
 */
class DDvsCtrl : public Ctrl 
{
public:
  DDvsCtrl(ros::NodeHandle &nh) : Ctrl(nh), useppv_(1) {
    ROS_INFO_STREAM("DDVS CREATED " << param );
    readParam();};

  ~DDvsCtrl();
  
  // matlab line [v_c(:,iter),e(:,iter)] = dvsPlanar(Gc, K, lambda, pt);
  int doCtrl(const Mat &G, const Mat &K, const Mat &pt, esmocv::Dvs_Status &dvsstat);
  
  int readParam() {
    Ctrl::readParam();
    nh_.param("ppv", useppv_, 1);
  }
  
private:
  int useppv_;/// chooses between two variants of the gain matrix M. 
}; // class

} //namespace

#endif