#include "sockutil/dvsutil.h"

using namespace dvsutil;

/**
 * \brief normalizes to get unhomogeneus values. 
 * \par divides each line by the last line, i.e., it works with collumn vectors
 */
 void Ut::unhomo(cv::Mat &M) {
  for (int i=0; i < M.rows ; i++)
    M.row(i) = M.row(i)/M.row(M.rows-1); // per element division
} // unhomo

/**
 * \brief copy a Mat COLUMN 6x1 matrix to a boost::array of 6 doubles
 * \par works with collumn vectors
 */
 void Ut::copyMat2Array(const cv::Mat &M, boost::array<double,6> &A) {
    for( int i=0; i < M.rows ; i++ )
      A[i] = M.at<double>(i,0);
}//copyMat2Array

/**
 * \brief copy a a boost::array of 6 doubles to a Mat COLUMN 6x1 matrix  
 * \par works with collumn vectors
 */
 void Ut::copyArray2Mat(const boost::array<double,6> &A, cv::Mat &M) {
    for( int i=0; i < M.rows ; i++ )
      M.at<double>(i,0) = A[i];
}//copyMat2Array


///Extracts the vetor v associated to an antisymmetric matrix A
cv::Mat Ut::vex(const cv::Mat &A) {
  //% Extracts the vetor v associated to an antisymmetric matrix A
  //matlab v = 0.5 * [A(3,2)-A(2,3); A(1,3)-A(3,1); A(2,1)-A(1,2)];
  cv::Mat v =  0.5 * (cv::Mat_<double>(3,1) << 
		      A.at<double>(2,1) - A.at<double>(1,2),
		      A.at<double>(0,2) - A.at<double>(2,0),
		      A.at<double>(1,0) - A.at<double>(0,1));
  return v;
}//vex

cv::Mat Ut::ppv(const cv::Mat &v) {
/*
function S = ppv(v)
S = [  0   -v(3)  v(2) ; 
      v(3)   0   -v(1) ;
     -v(2)  v(1)   0  ];
return % function
*/
  cv::Mat r =  (cv::Mat_<double>(3,3) << 0, -v.at<double>(2), v.at<double>(1),
				  v.at<double>(2), 0 , -v.at<double>(0),
				  -v.at<double>(1),v.at<double>(0),0);
  return r;
}//ppv


/// \brief prints out info about the opencv instalation
void Ut::openCVStatus(int printbuild ) {
  ROS_INFO("OpenCV Optimized: %s",(cv::useOptimized()?"true":"false") );
  ROS_INFO("OpenCV CPUs: %d, Threads: %d",cv::getNumberOfCPUs(),cv::getNumThreads() );
  
  ROS_INFO("OpenCV MMX: %s",(cv::checkHardwareSupport(CV_CPU_MMX)?"true":"false") );
  ROS_INFO("OpenCV SSE4_2: %s",(cv::checkHardwareSupport(CV_CPU_SSE4_2)?"true":"false") );
  ROS_INFO("OpenCV AVX: %s",(cv::checkHardwareSupport(CV_CPU_AVX)?"true":"false") );
  if (printbuild)
    ROS_INFO("OpenCV Build: %s",cv::getBuildInformation().c_str());
}

/** \brief tries to set the scheduler to FIFO
 *  \TODO UNTESTED!!!
  * \param nice the priority will be max_priority - nice. Do NOT abuse setting everybody to nice=0
  * 
  * \par about the setsheduler function from
  * \par http://man7.org/linux/man-pages/man2/sched_setscheduler.2.html  
  * \par The scheduling policy and parameters are in fact per-thread attributes on Linux.  The value returned from a call to gettid(2) can be passed in the argument pid.  Specifying pid as 0 will operate on the attribute for the calling thread, and passing the value returned       from a call to getpid(2) will operate on the attribute for the main thread of the thread group.  (If you are using the POSIX threads API, then use pthread_setschedparam(3), pthread_getschedparam(3), andpthread_setschedprio(3), instead of the sched_*(2) system calls.)
*/
int Ut::goRealTime(int nice) {
  if (SCHED_FIFO == sched_getscheduler (0)) { 
    ROS_WARN_THROTTLE (10,"Ut::goRealTime(): why go RT if it is already RT? did nothing"); 
    return 0; 
  };
  struct sched_param proc_sched;
  if (sched_getparam(0, &proc_sched)) { 
    ROS_ERROR ("Ut::goRealTime() Could not get thread parameters: %s", strerror (errno));
    return errno;
  }
  // change the priority to almost the maximum one. bigger values = more priority
  proc_sched.sched_priority = sched_get_priority_max(SCHED_FIFO)-nice;
  // set FIFO scheduler, 0 as the first parameter means the current thread!
  if (0 == sched_setscheduler (0, SCHED_FIFO, &proc_sched)) {
      ROS_INFO_THROTTLE (10,"Ut::goRealTime(): set RT sched with prio %d",proc_sched.sched_priority);
  } else {
      ROS_ERROR ("Ut::goRealTime() failed to set real-time scheduler: %s", strerror (errno));
      return errno;
  }
  
  return 0;
}//goRealTime


/// \brief given a 3xn matrix of homogeneous 2d points, one per collumn, calculate their bounding box 
cv::Rect Ut::boundingBoxOf2DPoints(cv::Mat &m) {
    dvsutil::Ut::unhomo(m); //unhomogeneous...
    // copy the points into a vector of cv::Point, boundingRect needs this
    std::vector<cv::Point> aux(m.cols);// integer points...
    for (int c = 0; c < m.cols ; c++) {
      aux[c].x = round(m.at<float>(0,c));
      aux[c].y = round(m.at<float>(1,c));
    }
    return cv::boundingRect(aux); 
}//boundingBoxOf2DPoints