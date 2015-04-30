/*
  * Class TelemetricOdometry unsing PSM 
  * 
  * Author: R. J. Martins
  * 
  * Date: January 2011
  */

#ifndef _TELEODOM_
#define _TELEODOM_

#include <iostream>
#include <cmath>
#include <stdio.h>

namespace laserodometry
{
#ifndef M_PI
#define M_PI 3.1415926535
#endif

#ifndef INFINITY
#define INFINITY 1e+10
#endif

#define isfinite(x) ((x) < 1e+10)

#define TYPE_TELEMETRIC            double
#define N		720
#define M 		1

class TelemetricDefines {
public:
	static const double MAX_ANGLE;
	static const double RESOLUTION;
	static const int HALF_WINDOW;
	static const int WINDOW;
	static const int SEGMENT_HOP;
	static const int MAX_RANGE;
	//static const int MAX_ITERATIONS;
	static const int MAX_ITERATIONS;
	static const int MAX_SMALL_STEPS;
	static const int PSM_WINDOW;
	static const double MIN_RANGE;      
	static const double MAX_DELTA;       
	static const int MIN_POINTS;   
	static const double SMALL_STEP;      
	static const double EPSILON;                   
	//static const int MIN_SEGMENT;   
	static const int MIN_SEGMENT;     
	static const double SEUIL_PREFILT;  
	static const double INVALID_RANGE;  
	
	static const int WIDTH;   
	static const int HEIGHT;        
	
	/* tags */
	static const int OUT_OF_RANGE;   
	static const int OCCLUDED;  
	static const int EMPTY;      
	
	static const double R2D;          
	static const double D2R;          
	

};


/* Laser scan representation */
typedef struct {
  /* scan pose */
  TYPE_TELEMETRIC x;
  TYPE_TELEMETRIC y;
  TYPE_TELEMETRIC theta;

  /* range measurements */
  TYPE_TELEMETRIC r[N];
  unsigned char tag[N];
  int segment[N];
} Scan;

typedef struct _laserDonnes { 
  double xy[N][2];
  int nelem;
} LASER_DONNEES;


class TelemetricOdometry 
{
public:

    TelemetricOdometry();

    
private:
  
    bool first_iteration;
    Scan l_temp;
    TYPE_TELEMETRIC T[4][4], U[4][4], V[4][4], T1[4][4],
    T2[4][4], T3[4][4], new_T3[4][4], T1inv[4][4],
    TLinv[4][4], Tginv[4][4], TL[4][4], Tgps[4][4];
    
    // Computing sinos and cossinos just once, since the number of samples and
    // resolutions are constants
    TYPE_TELEMETRIC Fi[N], Cos[N], Sin[N];
    
     
public:

    // Maths operations with homogeneous coordinates
    void GenerateMatrixT(TYPE_TELEMETRIC x, TYPE_TELEMETRIC y, TYPE_TELEMETRIC theta, TYPE_TELEMETRIC T[4][4]);
    void multiply(TYPE_TELEMETRIC A[4][4], TYPE_TELEMETRIC B[4][4], TYPE_TELEMETRIC C[4][4]);
    void invert(TYPE_TELEMETRIC A[4][4], TYPE_TELEMETRIC B[4][4]);
    
    // Filters scan laser points
    void median_filter(Scan * ls);
    void segment_scan(Scan * ls);
    void segment_filter(Scan * ls);
    
    // Store the last scan
    void store_Last_Scan(Scan *lr);
    // Read the last scan
    Scan load_Last_Scan();
    
    //void create_Struct_laser()
    void estimate_Laser_Odom(Scan *ls);
    // Estimate de Polar Matching    
    int psm(Scan *lsr, Scan *lsa);    
        
};

}

#endif

