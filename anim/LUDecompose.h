/**************************************************************************
 A modified version of Chandra Shekhar's LUDecompose algorithm
 This class calculates inverse of a 3 * 3 matrix
 Source: http://chandraacads.blogspot.ca/2015/12/c-program-for-matrix-inversion.html
***************************************************************************/
#include <shared/defs.h>
#include <util/util.h>


int LUPdecompose(double a[3][3], int p[3]);

int LUPinverse(int P[3], double LU[3][3], double B[3][3], double X[3], double Y[3]);

