/*
  An implementation of SVD from Numerical Recipes in C
*/

#ifndef SVD_H
#define SVD_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define SIGN(a,b) ((b) > 0.0 ? fabs(a) : - fabs(a))

static float maxarg1,maxarg2;
#define FMAX(a,b) (maxarg1 = (a),maxarg2 = (b),(maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))

static int iminarg1,iminarg2;
#define IMIN(a,b) (iminarg1 = (a),iminarg2 = (b),(iminarg1 < (iminarg2) ? (iminarg1) : iminarg2))

static float sqrarg;
#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg * sqrarg)

/*
  Modified from Numerical Recipes in C
  Given a matrix a[nRows][nCols], svdcmp() computes its singular value 
  decomposition, A = U * W * Vt.  A is replaced by U when svdcmp 
  returns.  The diagonal matrix W is output as a vector w[nCols].
  V (not V transpose) is output as the matrix V[nCols][nCols].
*/
int svdcmp(float **a, int nRows, int nCols, float *w, float **v);

// prints an arbitrary size matrix to the standard output
void printMatrix(float **a, int rows, int cols);

// prints an arbitrary size vector to the standard output
void printVector(float *v, int size);

// calculates sqrt( a^2 + b^2 ) with decent precision
float pythag(float a, float b);

#endif
