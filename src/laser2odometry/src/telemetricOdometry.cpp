#include "telemetricOdometry.h"

using namespace laserodometry;

// Constants
double const TelemetricDefines::MAX_ANGLE = 180.0;
double const TelemetricDefines::RESOLUTION = TelemetricDefines::MAX_ANGLE / N;
int const TelemetricDefines::HALF_WINDOW = 2;
int const TelemetricDefines::WINDOW = (2 * TelemetricDefines::HALF_WINDOW + 1);
int const TelemetricDefines::SEGMENT_HOP = 1;
int const TelemetricDefines::MAX_RANGE = 60;
//int const TelemetricDefines::MAX_ITERATIONS = 50;
int const TelemetricDefines::MAX_ITERATIONS = 150;
int const TelemetricDefines::MAX_SMALL_STEPS = 100;
int const TelemetricDefines::PSM_WINDOW = 20;
double const TelemetricDefines::MIN_RANGE = 0.0230000000447;
double const TelemetricDefines::MAX_DELTA = 1.0;
int const TelemetricDefines::MIN_POINTS = 5;
double const TelemetricDefines::SMALL_STEP = 0.01;
double const TelemetricDefines::EPSILON = 0.001;
//int const TelemetricDefines::MIN_SEGMENT = 7;
int const TelemetricDefines::MIN_SEGMENT = 3;
double const TelemetricDefines::SEUIL_PREFILT = 0.06;     /* in filtrageLaser.c */
double const TelemetricDefines::INVALID_RANGE =0.0;      /* in filtrageLaser.c */

int const TelemetricDefines::WIDTH = 900;      /* for opencv Windows */
int const TelemetricDefines::HEIGHT = 600;

/* tags */
int const TelemetricDefines::OUT_OF_RANGE = 1;
int const TelemetricDefines::OCCLUDED = 8;
int const TelemetricDefines::EMPTY = 16;

double const TelemetricDefines::R2D = (180.0 / M_PI);
double const TelemetricDefines::D2R = (M_PI / 180.0);

// Constructor
TelemetricOdometry::TelemetricOdometry(){
 
  first_iteration = true;  
  // precompute angles, sin and cos
  
  int i;
  for (i = 0; i < N; i++) {
    Fi[i] = i * TelemetricDefines::RESOLUTION * TelemetricDefines::D2R;
    Sin[i] = sin(Fi[i]);
    Cos[i] = cos(Fi[i]);
  }
  
  // Calibration between the laser and the car ABS
    /* int w
    for(i = 0; i < 4; i++)
    for(w = 0; w < 4; w++){
      if(i == w)
      {
	TL[i][w] = 1;
	Tgps[i][w] = 1;
      }else
      {
	TL[i][w] = 0;
	Tgps[i][w] = 0;
      }
    }  
 */
 
 /*Tgps = {{1,0,0,-0.3},
         {0,1,0,0},
         {0,0,1,0},
         {0,0,0,1}};*/
 Tgps[0][0] = 1;
 Tgps[0][1] = 0;
 Tgps[0][2] = 0;
 Tgps[0][3] = -0.3;
 Tgps[1][0] = 0;
 Tgps[1][1] = 1;
 Tgps[1][2] = 0;
 Tgps[1][3] = 0;
 Tgps[2][0] = 0;
 Tgps[2][1] = 0;
 Tgps[2][2] = 1;
 Tgps[2][3] = 0;
 Tgps[3][0] = 0;
 Tgps[3][1] = 0;
 Tgps[3][2] = 0;
 Tgps[3][3] = 1;
	 
 /*TL = {  {1,0,0,0},
         {0,1,0,0},
         {0,0,1,0},
         {0,0,0,1}};*/

 TL[0][0] = 1;
 TL[0][1] = 0;
 TL[0][2] = 0;
 TL[0][3] = 0;
 TL[1][0] = 0;
 TL[1][1] = 1;
 TL[1][2] = 0;
 TL[1][3] = 0;
 TL[2][0] = 0;
 TL[2][1] = 0;
 TL[2][2] = 1;
 TL[2][3] = 0;
 TL[3][0] = 0;
 TL[3][1] = 0;
 TL[3][2] = 0;
 TL[3][3] = 1;
 
  // Calibration between the car center and the GPS    
  Tgps[0][3] = -0.3;
    

//    Tgps = {{1,0,0,-0.3},
//                  {0,1,0,0},
//                  {0,0,1,0},
//                  {0,0,0,1}};
  
  invert(TL, TLinv);
  invert(Tgps, Tginv);  
}

// Save the last Scan 
void TelemetricOdometry::store_Last_Scan(Scan *lr){
 
l_temp = *lr;
  
}

// Load the stored scan
Scan TelemetricOdometry::load_Last_Scan(){
 
  return l_temp;
  
}

  

// Calculates the laser odometry
void TelemetricOdometry::estimate_Laser_Odom(Scan *lsa){
  
  TYPE_TELEMETRIC x_rr, y_rr, theta_rr, x_cr, y_cr, theta_cr;
  Scan lsr;
  int i,j;
  

  if(first_iteration){
    
    x_rr = lsa->x;
    y_rr = lsa->y;
    theta_rr = lsa->theta;
    GenerateMatrixT(x_rr,y_rr,theta_rr,T);
    
    // test if the range is valid
    for (i = 0; i < N; i++) {
      lsa->tag[i] = 0x00;
      if (lsa->r[i] <= TelemetricDefines::MIN_RANGE || lsa->r[i] >= TelemetricDefines::MAX_RANGE)
	lsa->tag[i] |= TelemetricDefines::OUT_OF_RANGE;
    }

    // filter data
    segment_scan(lsa);
    segment_filter(lsa);
    
    multiply(T, TL, T3);
    
    store_Last_Scan(lsa);
    
    first_iteration = false;
    
    lsa->x = T3[0][3];
    lsa->y = T3[1][3];
    lsa->theta = atan2f(T3[1][0], T3[1][1]);
 
    }
  else{
    
    lsr = load_Last_Scan();
    x_rr = lsr.x;
    y_rr = lsr.y;
    theta_rr = lsr.theta;
    // test if the range is valid
    for (i = 0; i < N; i++){
      lsa->tag[i] = 0x00;
      if (lsa->r[i] <= TelemetricDefines::MIN_RANGE || lsa->r[i] >= TelemetricDefines::MAX_RANGE)
	lsa->tag[i] |= TelemetricDefines::OUT_OF_RANGE;
    }
    
    // Filter current scan
    segment_scan(lsa);
    segment_filter(lsa);
    
    //Compute pose estimation according to odometry data
    //Now put errors here!
    /*x_cr = lsa->x +0.1;
    y_cr = lsa->y-0.1;
    theta_cr = lsa->theta + 0.07;
    */
    x_cr = lsa->x;
    y_cr = lsa->y;
    theta_cr = lsa->theta;
    
    GenerateMatrixT(x_rr,y_rr,theta_rr,T1);
    GenerateMatrixT(x_cr,y_cr,theta_cr,T2);
    invert(T1, T1inv);
    multiply(TLinv, T1inv, U);
    multiply(U, T2, V);
    multiply(V, TL, T);
    
    // Store actual scan to use it as refence in next cicle
    store_Last_Scan(lsa);

    //lsa.y = -T[0][3];  //y1 = -x0
    //lsa.x = T[1][3];   //x1 = y0
    lsa->x = T[0][3];
    lsa->y = T[1][3];
    lsa->theta = atan2f(T[1][0], T[1][1]);

    // Scan matching
    psm(&lsr, lsa);
 
    GenerateMatrixT(lsa->x,lsa->y,lsa->theta,new_T3);
    
    // Compose the new T3 with the previous T3
    multiply(T3, new_T3, T);
    
    // Update T3 
    for (i = 0; i < 4; i++)
      for (j = 0; j < 4; j++)
	T3[i][j] = T[i][j];
      
      
    lsa->x = T3[0][3];
    lsa->y = T3[1][3];
    lsa->theta = atan2f(T3[1][0], T3[1][1]);
	
 }
  
}



/* Polar scan matching with initial pose estimation (lsa->x,lsa->y,lsa->theta) */ 
int TelemetricOdometry::psm(Scan *lsr, Scan *lsa)
{
  TYPE_TELEMETRIC ax, ay, atheta;
  TYPE_TELEMETRIC dx, dy, dtheta;
  TYPE_TELEMETRIC r[N], fi[N], sampled_r[N];
  TYPE_TELEMETRIC C = 70 * 70;
  unsigned char tag[N];
  int i, iter; 
  TYPE_TELEMETRIC small_steps;
  TYPE_TELEMETRIC error_min = 1;
  TYPE_TELEMETRIC best_ax = lsa->x,best_ay = lsa->y,best_atheta=lsa->theta;

  ax = lsa->x;
  ay = lsa->y;
  atheta = lsa->theta;

  small_steps = -1;
  
  dx = 0;
  dy = 0;
  dtheta = 0;

  for(iter = 0; iter < TelemetricDefines::MAX_ITERATIONS && small_steps < TelemetricDefines::MAX_SMALL_STEPS; iter++) {

    /* Increase small correction count, if criteria is met */
    if (fabs(dx) + fabs (dy) + fabs(dtheta * TelemetricDefines::D2R) < TelemetricDefines::SMALL_STEP) {
      small_steps++;
    } else {
      small_steps = 0;
    }

    /* Convert range readings to reference scan frame */
    for (i = 0; i < N; i++) {
      TYPE_TELEMETRIC x = lsa->r[i] * cos(atheta + Fi[i]) + ax;
      TYPE_TELEMETRIC y = lsa->r[i] * sin(atheta + Fi[i]) + ay;

      r[i] = sqrt(x * x + y * y);
      fi[i] = atan2(y, x); 
      if (fi[i] < 0) fi[i] += 2 * M_PI;
      /* REMARK: to fit result in [0,2PI), add 2PI if it is negative */
      sampled_r[i] = TelemetricDefines::MAX_RANGE * 10;
      tag[i] = TelemetricDefines::EMPTY;
    }

    for (i = 1; i < N; i++) {
      bool occluded;
      int j0, j1;
      TYPE_TELEMETRIC a0, a1, r0, r1;

      if (lsa->segment[i] != 0 &&
      lsa->segment[i] == lsa->segment[i - 1] &&
      !lsa->tag[i] &&
      !lsa->tag[i - 1]) {
//  if(fi[i]-fi[i-1]>M_PI) fi[i-1]+= 2 * M_PI;
//  if(fi[i-1]-fi[i]>M_PI) fi[i]+= 2 * M_PI;

    if (fi[i] > 0 && fi[i - 1] >= 0) {
      if (fi[i] > fi[i - 1]) {
        occluded = false;
        a0 = fi[i - 1];
        a1 = fi[i];
        r0 = r[i - 1];
        r1 = r[i];  
      } else {
        occluded = true;
        a0 = fi[i];
        a1 = fi[i - 1];
        r0 = r[i];
        r1 = r[i - 1];      
      }

      j0 = ((int) ceil(a0 * TelemetricDefines::R2D / TelemetricDefines::RESOLUTION));
      j1 = ((int) (a1 * TelemetricDefines::R2D / TelemetricDefines::RESOLUTION));
      //assert(j0 <= j1);

          if(j0>j1) j0=j1;
//    if (j1 - j0 > N/4){printf("%d %d\n",j1, j0);j0=j1;}
          if (j1 - j0 >5){;j0=j1;}

      for( ; j0 <= j1 && j0<N; j0++) {
        /* interpolate to estimate a range */
        TYPE_TELEMETRIC ri = 
          (r1 - r0) / (a1 - a0) * ((TYPE_TELEMETRIC) j0 * TelemetricDefines::D2R * TelemetricDefines::RESOLUTION - a0) + r0;
        
        if (ri < sampled_r[j0%N]) {
          sampled_r[j0%N] = ri;
          tag[j0%N] &= ~TelemetricDefines::EMPTY;
          if (occluded)
             tag[j0%N] |= TelemetricDefines::OCCLUDED;
          else
             tag[j0%N] &= ~TelemetricDefines::OCCLUDED;
        }
      }  
    } else {
      /* TODO: take into account negative projected bearings */ 
    }
      }
    }

    if (iter == 10) C = 10*10; /* TODO: adjust this */

    if (iter % 5 == 1){

      /* Orientation estimation */

      TYPE_TELEMETRIC err[2 * TelemetricDefines::PSM_WINDOW + 1];
      TYPE_TELEMETRIC min_err;
      int beta[2 * TelemetricDefines::PSM_WINDOW + 1];

      int di, j, k;

      k = 0;
      for (di = -TelemetricDefines::PSM_WINDOW; di <= TelemetricDefines::PSM_WINDOW; di++) {
    int i, n, min_i, max_i;
    TYPE_TELEMETRIC e = 0;
    n = 0;

    /* compute bearing bracket */
    if (di <= 0) {
      min_i = -di;
      max_i = N;
    } else {
      min_i = 0;
      max_i = N - di;
    }
    
    /* compute the sum of the range residuals */
    for(i = min_i; i < max_i; i++) {
      if (!tag[i] && !lsr->tag[i + di] && sampled_r[i]<TelemetricDefines::MAX_RANGE && lsr->r[i+di]>0) {
        e += fabs(sampled_r[i] - lsr->r[i + di]);
        n++;
      }
    }
    
    if (n > 0)
      err[k] = e / n;
    else
      err[k] = INFINITY;

    beta[k] = di;
    k++;
      }

      /* Find the orientation with the minimum error */
      min_err = INFINITY;
      
      for (i = 0; i < k; i++) {
    if (err[i] < min_err) {
      min_err = err[i];
      j = i;
    }
      }
      
      if (!isfinite(min_err)) {
//    fprintf(stderr, "psm: orientation search failed\n");
//  return -1;
      }

      dtheta = beta[j];      

      /* Compute orientation by quadratic interpolation between 
     the orientation with the minimum error and its two 
     closest neighbours */ 
      /* TODO: try removing this */
      if(1 <= j && j < k - 1) {
        TYPE_TELEMETRIC den = 2 * err[j] - err[j - 1] - err[j + 1];
        TYPE_TELEMETRIC m;

        if(fabs(den) > TelemetricDefines::EPSILON) {
          m = (err[j + 1] - err[j - 1]) / (2 * den);
      if(fabs(m) < 1)
        dtheta += m;

    }
    }
     
      atheta += dtheta * TelemetricDefines::D2R * TelemetricDefines::RESOLUTION;

    }
    else {

      /* Translation estimation */

      TYPE_TELEMETRIC HTWH[2][2];
      TYPE_TELEMETRIC HTWHinv[2][2];
      TYPE_TELEMETRIC HTWr[2];

      HTWr[0] = 0;
      HTWr[1] = 0;
      
      HTWH[0][0] = 0;
      HTWH[0][1] = 0;
      HTWH[1][0] = 0;
      HTWH[1][1] = 0;

      int n = 0;
      TYPE_TELEMETRIC error=0;
      for (i = 0; i < N; i++) {
    TYPE_TELEMETRIC dr = lsr->r[i] - sampled_r[i];
    
    if (lsr->r[i]>0 && !lsr->tag[i] && !tag[i] &&
        sampled_r[i] <= TelemetricDefines::MAX_RANGE &&
        sampled_r[i] >= TelemetricDefines::MIN_RANGE &&
        fabs(dr) < TelemetricDefines::MAX_DELTA) {
      TYPE_TELEMETRIC w = C / (dr * dr + C);
      TYPE_TELEMETRIC h1 = Cos[i];
      TYPE_TELEMETRIC h2 = Sin[i];
     
      HTWr[0] += h1 * w * dr;
      HTWr[1] += h2 * w * dr;

      HTWH[0][0] += w * h1 * h1;
      HTWH[0][1] += w * h1 * h2;
      HTWH[1][0] += w * h2 * h1; /* TODO: use HTWH[0][1] instead */
      HTWH[1][1] += w * h2 * h2;
          error+=w*dr*dr;
      n++;
    }
      }
      if (n >= TelemetricDefines::MIN_POINTS) printf("%d\n", iter);
      if (n < TelemetricDefines::MIN_POINTS) {
      	      //stringstream ss;
      	      fprintf(stderr,"NOT ENOUGH LASER POINTS O que fazer? %d\n", iter);
    	      break;
      }

      error=error/(TYPE_TELEMETRIC)n;

//       fprintf(stderr, "error:%f\n", error);
      if(error<error_min) {
          error_min=error;
          best_ax=ax;
          best_ay=ay;
          best_atheta=atheta;
      }

      TYPE_TELEMETRIC det = HTWH[0][0] * HTWH[1][1] - HTWH[0][1] * HTWH[1][0];

      if(fabs(det) < TelemetricDefines::EPSILON) {
//    fprintf(stderr, "psm: determinant too small in translation estimation\n");
    return n;
      }

      HTWHinv[0][0] = HTWH[1][1] / det;
      HTWHinv[0][1] = -HTWH[0][1] / det;
      HTWHinv[1][0] = -HTWH[1][0] / det;
      HTWHinv[1][1] = HTWH[0][0] / det;

      dx = HTWHinv[0][0] * HTWr[0] + HTWHinv[0][1] * HTWr[1];
      dy = HTWHinv[1][0] * HTWr[0] + HTWHinv[1][1] * HTWr[1]; 

      ax += dx;
      ay += dy;
  
    }
    
  }
// if(error_min<0.003){

  lsa->x = best_ax;
  lsa->y = best_ay;
  lsa->theta = best_atheta;
//  fprintf(stderr, "error:%f\n", error_min);
// }else{
//   fprintf(stderr, "error_:%f\n", error_min);
//  }
  return 0;
}

// Multiplies A * B, storing the result in C 
void TelemetricOdometry::multiply(TYPE_TELEMETRIC A[4][4], TYPE_TELEMETRIC B[4][4], TYPE_TELEMETRIC C[4][4]) {
  int i, j, k;
    	
  for (i = 0; i < 4; i++) {
    for (j = 0; j < 4; j++) {
      C[i][j] = 0;
      for (k = 0; k < 4; k++) {
	C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

void TelemetricOdometry::segment_filter(Scan *ls){
  int seg_c=-1;
  int n=0,first_in_seg;
  int i;
  for(i=0;i<N;i++){  
    if(ls->r[i]==0) {ls->segment[i]=0;/*ls->tag[i]&=~TelemetricDefines::EMPTY;ls->tag[i]|=TelemetricDefines::OUT_OF_RANGE;*/}

    if(seg_c==-1){
// printf("inicio segmento: %d\n",i);
      seg_c=ls->segment[i];
      first_in_seg=i;
   }else
    if(seg_c != ls->segment[i]){
// printf("fin segmento: %d\n",i-1);
      if(n<TelemetricDefines::MIN_SEGMENT){
          int j=first_in_seg;
          for(;j<i;j++){
//                 printf("elimina punto: %d\n",j);
                ls->r[j]=0;
                ls->segment[j]=0; /*ls->tag[j]&=~TelemetricDefines::EMPTY;ls->tag[i]|=TelemetricDefines::OUT_OF_RANGE;*/
          }
      }
      n=0;
// printf("inicio segmento: %d\n",i);
      seg_c=ls->segment[i];
      first_in_seg=i;
  }
//   printf("%d\n",i);
  n++;

}

}

// segment scan laser points
void TelemetricOdometry::segment_scan(Scan * ls)
{
  int i, n, m;
  
  /* segment number */
  n = 1;

  if (fabs(ls->r[0] - ls->r[1]) < TelemetricDefines::SEGMENT_HOP) {
    ls->segment[0] = 1;
    ls->segment[1] = 1;
    m = 2;
  } else {
    ls->segment[0] = 0;
    ls->segment[1] = 1;
    m = 1;
  }

  for (i = 2; i < N; i++) {
    bool next = false;

    if (ls->tag[i]) {
      /* tagged measurements break segments */
      next = true;
      ls->segment[i] = 0;
    } else {
      /* TODO: find out from where does this come */
      TYPE_TELEMETRIC delta = ls->r[i] - (2.0 * ls->r[i - 1] - ls->r[i - 2]);
      if (fabs(ls->r[i] - ls->r[i - 1]) < TelemetricDefines::SEGMENT_HOP ||
      (ls->segment[i - 1] == ls->segment[i - 2] &&
       fabs(delta) < TelemetricDefines::SEGMENT_HOP)) {
    m++;
    ls->segment[i] = n;
      } else {
    next = true;
      }
    }

    if (next) {
      if (m == 1) {
    TYPE_TELEMETRIC delta = ls->r[i] - (2.0 * ls->r[i - 1] - ls->r[i - 2]);
    if (ls->segment[i - 2] == 0 &&
        !ls->tag[i] && !ls->tag[i - 1] && !ls->tag[i - 2] &&
        fabs(delta) < TelemetricDefines::SEGMENT_HOP) {
      ls->segment[i] = n;
      ls->segment[i - 1] = n;
      ls->segment[i - 2] = n;
      m = 3;
    } else {
      ls->segment[i - 1] = 0;
      ls->segment[i] = n;
      m = 1;
    }
      } else {
    n++;
    ls->segment[i] = n;
    m = 1;
      }
    }
  }
}

void TelemetricOdometry::GenerateMatrixT(TYPE_TELEMETRIC x, TYPE_TELEMETRIC y, TYPE_TELEMETRIC theta, TYPE_TELEMETRIC T[4][4]){
    T[0][0] = cos(theta);
    T[0][1] = -sin(theta);
    T[0][2] = 0;
    T[0][3] = x;

    T[1][0] = sin(theta);
    T[1][1] = cos(theta);
    T[1][2] = 0;
    T[1][3] = y;
    
    T[2][0] = 0;
    T[2][1] = 0;
    T[2][2] = 1;
    T[2][3] = 0;
    
    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
}


// Inverse of a rigid transformation
void TelemetricOdometry::invert(TYPE_TELEMETRIC A[4][4], TYPE_TELEMETRIC B[4][4]) {
  int i, j;

  for (i = 0; i < 3; i++) {
        B[i][3] = 0;
        for (j = 0; j < 3; j++) {
                B[i][j] = A[j][i];
                B[i][3] -= A[j][i] * A[j][3];
        }
  }

  B[3][0] = 0;
  B[3][1] = 0;
  B[3][2] = 0;
  B[3][3] = 1;
}
