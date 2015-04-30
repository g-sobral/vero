
#ifndef DVSUTIL_UT_H
#define DVSUTIL_UT_H


#include <math.h>       /* round, floor, ceil, trunc */
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>/// CV_CPU constants are here.
#include <opencv2/imgproc/imgproc.hpp>/// boundingrect
#include <boost/array.hpp>
#include <ros/ros.h>
#include <errno.h> /// handling errors on scheduler funciots
#include <sched.h> ///scheduling funtions

///////////////// macros to unit conversion.////////////////
#define utRAD2DEG(radians) ((radians) * (180.0 / M_PI))
#define utDEG2RAD(degrees) ((degrees) * (M_PI / 180.0))

////////////////// gearbox macros /////////////////////

// M_PI is not defined after including cmath for the MS visual studio compiler?
#ifndef M_PI
//dox Defines number Pi
#define M_PI 3.14159265358979323846
#endif

#ifndef DEG2RAD_RATIO
//dox Convertion factor from degrees to radians.
#define DEG2RAD_RATIO	(M_PI/180.0)
#endif

//dox Converts from degrees to radians.
#define DEG2RAD(deg)	((deg)*DEG2RAD_RATIO)
//dox Converts from radians to degrees.
#define RAD2DEG(rad) 	((rad)/DEG2RAD_RATIO)

#ifndef MIN
//dox Minimum of two numbers
#define MIN(x, y)        (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
//dox Maximum of two numbers
#define MAX(x, y)        (((x) > (y)) ? (x) : (y))
#endif

namespace dvsutil {


/**
 * \brief Utility class with math and data copying stuff for dvs related software.
 * \par a Java.Math for dvs :-) Be a nice guy and grow this lib instead of keeping functions scattered though your code.
 */
class Ut {
public:
  static void unhomo(cv::Mat &M) ;

  static void copyMat2Array(const cv::Mat &M, boost::array<double,6> &A) ;
  static void copyArray2Mat(const boost::array<double,6> &A, cv::Mat &M) ;
  static cv::Rect boundingBoxOf2DPoints(cv::Mat &m) ;
  
  static int goRealTime(int nice);
    
  // matlab function v = vex(A)
  static cv::Mat vex(const cv::Mat &A);
  static cv::Mat ppv(const cv::Mat &v);

  static void openCVStatus(int printbuild = 0);
  
};//class


};// namespace 
  
  
#endif
