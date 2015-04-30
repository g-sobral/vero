#include <iostream>
#include <stdio.h>
#include <math.h>
//#include <cv.h> // already included in filter.h
#include "filter.h"

 namespace fusion2localise2d {

 Filter::Filter()  {

 	isInitialized = false;

 	q1 = 0.02; //cfg_.???;  incerteza da odometria = 0.02
	r1 = 5;  //cfg_.???;  incerteza do gps = 5

	xp.create(3,1,CV_64FC1);
	Q.create(3,3,CV_64FC1);
	R.create(3,3,CV_64FC1);
	p0.create(3,3,CV_64FC1);
	ma0.create(9,1,CV_64FC1);
	pa.create(9,9,CV_64FC1);
	xa.create(9,1,CV_64FC1);
	Wm.create(1,19,CV_64FC1);
	Wc.create(1,19,CV_64FC1);
	yest.create(3,19,CV_64FC1);
	xest.create(3,19,CV_64FC1);
 	X.create(9,19,CV_64FC1);

 }


 cv::Mat Filter::f (cv::Mat z,cv::Mat s,cv::Mat tv) {

 	cv::Mat ret(3,1,CV_64FC1); ;

 	ret.at<double>(0,0) = z.at<double>(0,0) + tv.at<double>(0,0) + s.at<double>(0,0)*cos(z.at<double>(2,0) + tv.at<double>(2,0) + s.at<double>(1,0)/2);
 	ret.at<double>(1,0) = z.at<double>(1,0) + tv.at<double>(1,0) + s.at<double>(0,0)*sin(z.at<double>(2,0) + tv.at<double>(2,0) + s.at<double>(1,0)/2);
 	ret.at<double>(2,0) = z.at<double>(2,0) + tv.at<double>(2,0) + s.at<double>(1,0);

  	return ret;

 }

 cv::Mat Filter::g (cv::Mat l,cv::Mat tn) {

 	cv::Mat ret(3,1,CV_64FC1); ;

 	ret.at<double>(0,0) = l.at<double>(0,0) + tn.at<double>(0,0);
 	ret.at<double>(1,0) = l.at<double>(1,0) + tn.at<double>(1,0);
 	ret.at<double>(2,0) = l.at<double>(2,0) + tn.at<double>(2,0);

  	return ret;

 }

 cv::Mat Filter::sigmas2(cv::Mat A, cv::Mat B) {

	cv::Mat ret(9,19,CV_64FC1);
	cv::Mat C(9,9,CV_64FC1);
	cv::Mat aux(9,9,CV_64FC1);
	cv::Mat aux2(9,9,CV_64FC1);
        cv::Mat aux3(9,9,CV_64FC1);
	int n = 9, i, j, k, l;
	double sum, alpha, lambda, c;
	
	l = n;
	alpha = 0.001;
	lambda = pow(alpha,2)*l - l;
	c = sqrt(l + lambda);
	
	for (i=0;i<n;i++){
	  for (j=0;j<n;j++){
	    C.at<double>(i,j) = 0;
	  };
	};
	
	for (i = 0; i < n; i++){
	  for (j = i;j < n; j++){
	    for (sum = B.at<double>(i,j), k = i-1; k >= 0; k--) sum -= B.at<double>(i,k)*B.at<double>(j,k);
	    if (i==j && sum<=0) {
	      //printf("NON-POSITIVE DEFINITE MATRIX\n\n");
	      //return eye(9,9,CV_64FC1)*(-1);
	      return C.eye(9,9,CV_64FC1)*(-1);
	    }
	    if (i == j)
	      C.at<double>(i,i) = sqrt(sum);
	    else C.at<double>(j,i) = sum/C.at<double>(i,i);
	  }
	}
	
	C = C*c;          //completeSymm(C, true);  for the symmetric part
	repeat(A,1,9,aux);
        add(aux,C,aux2);
        subtract(aux,C,aux3);
        //TODO
	for (i = 0; i < n; i++){
	   for (j = 0;j < (2*n + 1); j++){
	   	if(j == 0)
	   	   ret.at<double>(i,j) = A.at<double>(i,j);
	   	else if(j < 10)
	   	   ret.at<double>(i,j) = aux2.at<double>(i,j-1);
	   	else
	   	   ret.at<double>(i,j) = aux3.at<double>(i,j-10); 	
	   }
	}

	return ret;  

      }

//  double Filter::rem(double a, double b) {
// 
// 	// This is not a general implementation of Matlab's rem, but it's enough for our purposes.
// 
// 	if (a<0) {
// 		a*= -1;
// 		return (-1* (a%b));
// 	}
// 
// 	else 	return a%b;
// 
//  }

void Filter::printMat(cv::Mat A)
{
int i, j;
for (i = 0; i < A.rows; i++)
{
printf("\n");

for (j = 0; j < A.cols; j++)
printf ("%8.3f ", A.at<double>(i,j));

}
printf("\n");
}

 double Filter::rem(double a, double b) {
   double aux;
   
   aux = fmod(a,b);
   
   if (aux > PI)
      aux = -2*PI + aux;
   else if (aux < -PI)
           aux = 2*PI + aux; 
   return aux;
  }
  
 bool Filter::isInit () { return isInitialized; }

 double Filter::getFusionX () {

	return 	xp.at<double>(0,0);

 }

 double Filter::getFusionY () {

	return 	xp.at<double>(1,0);

 }

 double Filter::getFusionO () {

	return 	xp.at<double>(2,0);

 }

double Filter::getFusionxx () {

	return 	p0.at<double>(0,0);

 }

double Filter::getFusionyy () {

	return 	p0.at<double>(1,1);

 }

double Filter::getFusionxy () {

	return 	p0.at<double>(0,1);

 }

double Filter::getFusionxt () {

	return 	p0.at<double>(0,2);

 }

double Filter::getFusionyt () {

	return 	p0.at<double>(1,2);

 }

double Filter::getFusiontt () {

	return 	p0.at<double>(2,2);

 }


 void Filter::initialize (double gpsX, double gpsY, double gpsO) {
 
 cv::Mat aux(3,1,CV_64FC1);
 cv::Mat aux1(3,3,CV_64FC1);
 int i,j;


	xp.at<double>(0,0) = gpsX;
	xp.at<double>(1,0) = gpsY;
	xp.at<double>(2,0) = gpsO;

	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			if (i==j && i==2) 	Q.at<double>(i,j) = pow(q1,2)/4;
			else if (i==j)		Q.at<double>(i,j) = pow(q1,2);
			else 			Q.at<double>(i,j) = 0;
		}
	}
	
	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			if (i==j && i==2) 	R.at<double>(i,j) = pow(PI/6,2)/4;
			else if (i==j)		R.at<double>(i,j) = pow(r1,2);
			else 			R.at<double>(i,j) = 0;
		}
	}
        
	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			if (i==j && i==2) 	p0.at<double>(i,j) = 400*0.001;
			else if (i==j)		p0.at<double>(i,j) = 6250*0.001;
			else 			p0.at<double>(i,j) = 0;
		}
	}
        
						ma0.at<double>(0,0) = gpsX;
						ma0.at<double>(1,0) = gpsY;
						ma0.at<double>(2,0) = gpsO;
	for (i=3;i<9;i++) 			ma0.at<double>(i,0) = 0;

						xa = ma0;	

	//for (i=0;i<3;i++)
	//		for (j=0;j<3;j++)
	//					aux1.at<double>(i,j) = 0;

/*	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			if (i==j && i==0) 	pa.at<cv::Mat>(i,j) = p0;
			else if (i==j && i==1)	pa.at<cv::Mat>(i,j) = Q;
			else if (i==j && i==2)	pa.at<cv::Mat>(i,j) = R;
			else 			pa.at<cv::Mat>(i,j) = aux1;
		}
	}
*/
        

/*	 ************ Definindo constantes:	************ 	*/

	L = 9;
	alpha = 0.001;
	beta = 2;
	lambda = pow(alpha,2)*L-L;

	w0m = lambda/(L+lambda);
	w0c = w0m + (1-pow(alpha,2) + beta);

					
	for (i=1;i<2*L+1; i++){
		Wm.at<double>(0,i) = 1/(2*(L+lambda));
		Wc.at<double>(0,i) = 1/(2*(L+lambda));
 	    }	

	Wm.at<double>(0,0) = w0m;							
	Wc.at<double>(0,0) = w0c;


	isInitialized = true;
	cout << "Inicialization ok" <<endl;
}



 void Filter::pred_ukf (double odomX, double odomY, double odomO) {

  int n = 9,i,j,k;
  float sig1,sig2;
  cv::Mat    auxiliar1(3,1,CV_64FC1);
  cv::Mat    auxiliar2(3,1,CV_64FC1);
  cv::Mat    auxiliar3(2,1,CV_64FC1);
  cv::Mat    auxiliar4(3,1,CV_64FC1);
  cv::Mat    auxiliar5(3,1,CV_64FC1);
  cv::Mat    aux1(3,3,CV_64FC1);
  
        
  	if (!(isInitialized)) {

	 	lastXodom = odomX;
		lastYodom = odomY;
		lastOodom = odomO;                
  		return;
  	}
 	dth = sqrt(pow(odomY-lastYodom,2) + pow(odomX-lastXodom,2));
 	ath = odomO - lastOodom; 
 		
 	//find if is going forward or not...
 	i = atan2(odomY-lastYodom,odomX-lastXodom);
 	if ( fabs( fabs(i) - fabs( fmod(lastOodom + ath/2/* ) */,2*PI) ) ) > PI/2      ){ //Moved Parentheses because fmod had 1 arg and fabs had 2, 29.04.2104
 	 	dth = dth*(-1);
 	}
 	
 	lastXodom = odomX;
	lastYodom = odomY;
	lastOodom = odomO;
	
	for (i=0;i<3;i++) {
		for (j=0;j<2*n+1;j++) {
					yest.at<double>(i,j) = 0;
					xest.at<double>(i,j) = 0;
		}
	}
	
      sig1 = 0.05*dth + 1e-4*(dth == 0);
      sig2 = 0.122*ath + 1e-3*(ath == 0); 		
//******************************************************
 	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			if (i==j && i==2) 	Q.at<double>(i,j) = pow(sig2,2);
			else if (i==j)		Q.at<double>(i,j) = pow(sig1,2);
			else 			Q.at<double>(i,j) = 0;
		}
	}


	for (i=0;i<9;i++) for (j=0;j<9;j++) pa.at<double>(i,j) = 0; // pa = zeros(9)

	for (i=0;i<9;i++) {
		for (j=0;j<9;j++) {
			if (i < 3 && j < 3) 	
                           pa.at<double>(i,j) = p0.at<double>(i,j);
                        else if (i < 6 && j < 6 && i > 2 && j > 2 )
			   pa.at<double>(i,j) = Q.at<double>(i-3,j-3);			 
			else if (i >= 6 && j >= 6)
		           pa.at<double>(i,j) = R.at<double>(i-6,j-6);			
		}
	}

						xa.at<double>(0,0) = xp.at<double>(0,0);
						xa.at<double>(1,0) = xp.at<double>(1,0);
						xa.at<double>(2,0) = rem(xp.at<double>(2,0),2*PI);
	for (i=3;i<9;i++) 			xa.at<double>(i,0) = 0;

//*******************************************************
	X = sigmas2(xa,pa);
	
	/*TODO
 	if compare(X,eye(.)(-1)){
 	  return;
 	}*/

	auxiliar3.at<double>(0,0) = dth;
	auxiliar3.at<double>(1,0) = ath;

	for (j=0;j<2*n+1;j++){

		for (k=0;k<3;k++) auxiliar2.at<double>(k,0) = X.at<double>(k,j);
		for (k=3;k<6;k++) auxiliar4.at<double>(k-3,0) = X.at<double>(k,j);

		auxiliar1 = f(auxiliar2,auxiliar3,auxiliar4);
		for (k=0;k<3;k++) xest.at<double>(k,j) = auxiliar1.at<double>(k,0);

	}
	
	for (k=0;k<3;k++) xp.at<double>(k,0) = 0;

	for (j=0;j<2*n+1;j++){

		for (k=0;k<3;k++) auxiliar5.at<double>(k,0) = xest.at<double>(k,j);
		xp = xp + Wm.at<double>(0,j)*auxiliar5;

	}
	
 	for (i=0;i<3;i++)
 		for (j=0;j<3;j++)
 			p0.at<double>(i,j) = 0;


 	for (j=0;j<2*n+1;j++){

		for (k=0;k<3;k++) auxiliar5.at<double>(k,0) = xest.at<double>(k,j);
		p0 = p0 + Wc.at<double>(0,j)*(auxiliar5-xp)*((auxiliar5-xp).t());

	}	
	
 }


 void Filter::updateWithGPS (double gpsX, double gpsY, double gpsO){

  cv::Mat    auxiliar1(3,1,CV_64FC1);
  cv::Mat    auxiliar2(3,1,CV_64FC1);
  cv::Mat    auxiliar3(3,1,CV_64FC1);
  cv::Mat    auxiliar4(3,1,CV_64FC1);
  cv::Mat    ym(3,1,CV_64FC1);
  cv::Mat    p1(3,3,CV_64FC1);
  cv::Mat    p1_inv(3,3,CV_64FC1);
  cv::Mat    pxy(3,3,CV_64FC1);
  cv::Mat    kmat(3,3,CV_64FC1);
  cv::Mat    y(3,1,CV_64FC1);
  cv::Mat    toDM(1,1,CV_64FC1);
  int j,k,n=9,blue;
  double d_m; //quad;

        if (isInitialized == false) {
	     initialize(gpsX,gpsY,gpsO);
	     return;
	     }
	     	     
	for (j=0;j<2*n+1;j++){

		for (k=0;k<3;k++) auxiliar2.at<double>(k,0) = xest.at<double>(k,j);
		for (k=6;k<9;k++) auxiliar3.at<double>(k-6,0) = X.at<double>(k,j);

		auxiliar1 = g(auxiliar2,auxiliar3);

		for (k=0;k<3;k++) yest.at<double>(k,j) = auxiliar1.at<double>(k,0);
	}
	
	for (k=0;k<3;k++) ym.at<double>(k,0) = 0;

	for (j=0;j<2*n+1;j++){

		for (k=0;k<3;k++) auxiliar1.at<double>(k,0) = yest.at<double>(k,j);
		ym = ym + Wm.at<double>(0,j)*auxiliar1;
	}
	
	for (j=0;j<3;j++) for (k=0;k<3;k++) p1.at<double>(j,k) = 0;

	for (j=0;j<2*n+1;j++){

		for (k=0;k<3;k++) auxiliar1.at<double>(k,0) = yest.at<double>(k,j);
		p1 = p1 + Wc.at<double>(0,j)*(auxiliar1-ym)*((auxiliar1-ym).t());
	}
		
	for (j=0;j<3;j++) for (k=0;k<3;k++) pxy.at<double>(j,k) = 0;

	for (j=0;j<2*n+1;j++){

		for (k=0;k<3;k++) auxiliar1.at<double>(k,0) = xest.at<double>(k,j);
		for (k=0;k<3;k++) auxiliar2.at<double>(k,0) = yest.at<double>(k,j);

		pxy = pxy + Wc.at<double>(0,j)*(auxiliar1-xp)*( (auxiliar2-ym).t() );
	}
               
        invert(p1,p1_inv,cv::DECOMP_SVD);
     	
	kmat = pxy*p1_inv;
		
	y.at<double>(0,0) = gpsX;
	y.at<double>(1,0) = gpsY;
	y.at<double>(2,0) = gpsO;

	auxiliar4.at<double>(0,0) = y.at<double>(0,0)-ym.at<double>(0,0);
	auxiliar4.at<double>(1,0) = y.at<double>(1,0)-ym.at<double>(1,0);
	auxiliar4.at<double>(2,0) = rem(y.at<double>(2,0)-ym.at<double>(2,0), 2*PI);
	
	toDM = (auxiliar4.t())*p1_inv*auxiliar4;
	d_m = sqrt( toDM.at<double>(0,0) );
	
	d_m = 0;
	
	if (d_m < 3) {
		xp = xp + kmat*auxiliar4;
                xp.at<double>(2,0) = rem(xp.at<double>(2,0),2*PI);
		p0 = p0 - kmat*p1*kmat.t();
	}
	
 }

 void Filter::updateWithoutGPS (){

 //função vazia por enquanto...


}

 }
