
#ifndef FILTERODOMETRYGPS_H
#define FILTERODOMETRYGPS_H

#include <math.h>
#include <opencv/cv.h>


#define PI 3.1416

 using namespace std;

 namespace fusion2localise2d {

 class Filter {

 public:

	Filter();

 	double getFusionX ();
 	double getFusionY ();
 	double getFusionO ();
 	double getFusionxx ();
  	double getFusionyy ();
   	double getFusionxy ();
    	double getFusionxt ();
     	double getFusionyt ();
      	double getFusiontt ();
 	void pred_ukf (double odomX, double odomY, double odomO);
	void updateWithGPS (double gpsX, double gpsY, double gpsO);
	void updateWithoutGPS ();
	void printMat(cv::Mat A);
	bool isInit ();
 private:

	bool isInitialized;
	cv::Mat xp,Q,R,X,p0,ma0,pa,xa,Wm,Wc,yest,xest;
	double lastXodom, lastYodom, lastOodom;
	double dth,ath;
	double L,alpha,beta,lambda,w0m,w0c,q1,r1;

	void initialize (double gpsX, double gpsY, double gpsO);

	cv::Mat f (cv::Mat z,cv::Mat s,cv::Mat tv);
	cv::Mat g (cv::Mat l,cv::Mat tn);
	cv::Mat sigmas2(cv::Mat A, cv::Mat B);
	double rem(double a, double b);


 };

}

#endif

