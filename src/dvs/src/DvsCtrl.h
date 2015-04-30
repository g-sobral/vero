
#ifndef DVS_DVS_CTRL_H
#define DVS_DVS_CTRL_H

#include "Ctrl.h"

using namespace cv;

namespace dvs
{
  

/** \brief PURPOSE : direct visual servoing w.r.t. a planar object
 *
 * \par INPUTs 
 * \li - G in SL(3) is the projective homography from the reference frame to the camera frame
 * \li K in R^{3x3} is an estimate of the camera intrinsic params
 * \li lambda > 0 is the control gain
 * \li pt = [u,v,1]^T in P^2 is the control point
 *
 * \par OUPUT   : v_c in R^6 contains the camera velocities
 *
 * \par AUTHOR  :
 * Geraldo Silveira
 * http://sites.google.com/site/geraldofsilveira
 * Geraldo.Silveira@cti.gov.br
 *
 * \par REFERENCE : 
 * G. Silveira and E. Malis, "Direct Visual Servoing 
 * with respect to Rigid Objects," Proc. of IEEE/RSJ IROS,
 * USA, pp. 1963-1968, October 2007. 
 *
 * \par CTI Renato Archer, August 2010
 */
class DvsCtrl : public Ctrl 
{
public:
  DvsCtrl(ros::NodeHandle &nh) : Ctrl(nh) {
    readParam();
    ROS_INFO_STREAM("DVS CREATED " << param );
    };

  ~DvsCtrl();
  
  // matlab line [v_c(:,iter),e(:,iter)] = dvsPlanar(Gc, K, lambda, pt);
  int doCtrl(const Mat &G, const Mat &K, const Mat &pt, esmocv::Dvs_Status &dvsstat);
  
  int readParam() {Ctrl::readParam();}
  
private:
  
}; // class

} //namespace

#endif