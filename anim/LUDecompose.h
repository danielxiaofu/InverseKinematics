/**************************************************************************
 A modified version of Chandra Shekhar's LUDecompose algorithm
 This class calculates inverse of a 3 * 3 matrix
 Source: http://chandraacads.blogspot.ca/2015/12/c-program-for-matrix-inversion.html
***************************************************************************/
#include <shared/defs.h>
#include <util/util.h>


int LUPdecompose(double a[4][4], int p[4]);

int LUPinverse(int P[4], double LU[4][4], double B[4][4], double X[4], double Y[4]);

