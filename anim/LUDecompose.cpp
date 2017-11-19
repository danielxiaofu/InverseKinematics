#include "LUDecompose.h"

/* This function decomposes the matrix 'A' into L, U, and P. If successful,
* the L and the U are stored in 'A', and information about the pivot in 'P'.
* The diagonal elements of 'L' are all 1, and therefore they are not stored. */
int LUPdecompose(double A[4][4], int P[4])
{
	int i, j, k, kd = 0, T;
	double p, t;

	/* Finding the pivot of the LUP decomposition. */
	for (i = 1; i < 4; i++) P[i] = i; //Initializing.  

	for (k = 1; k < 3; k++)
	{
		p = 0;
		for (i = k; i < 4; i++)
		{
			t = A[i][k];
			if (t < 0) t *= -1; //Abosolute value of 't'.  
			if (t > p)
			{
				p = t;
				kd = i;
			}
		}

		if (p == 0)
		{
			return -1;
		}

		/* Exchanging the rows according to the pivot determined above. */
		T = P[kd];
		P[kd] = P[k];
		P[k] = T;
		for (i = 1; i < 4; i++)
		{
			t = A[kd][i];
			A[kd][i] = A[k][i];
			A[k][i] = t;
		}

		for (i = k + 1; i < 4; i++) //Performing substraction to decompose A as LU.  
		{
			A[i][k] = A[i][k] / A[k][k];
			for (j = k + 1; j < 4; j++) A[i][j] -= A[i][k] * A[k][j];
		}
	} //Now, 'A' contains the L (without the diagonal elements, which are all 1)  
	  //and the U.  

	return 0;
}

/* This function calculates the inverse of the LUP decomposed matrix 'LU' and pivoting
* information stored in 'P'. The inverse is returned through the matrix 'LU' itself.
* 'B', X', and 'Y' are used as temporary spaces. */
int LUPinverse(int P[4], double LU[4][4], double B[4][4], double X[4], double Y[4])
{
	int i, j, n, m;
	double t;

	//Initializing X and Y.  
	for (n = 1; n < 4; n++) X[n] = Y[n] = 0;

	/* Solving LUX = Pe, in order to calculate the inverse of 'A'. Here, 'e' is a column
	* vector of the identity matrix of size 'size-1'. Solving for all 'e'. */
	for (i = 1; i < 4; i++)
	{
		//Storing elements of the i-th column of the identity matrix in i-th row of 'B'.  
		for (j = 1; j < 4; j++) B[i][j] = 0;
		B[i][i] = 1;

		//Solving Ly = Pb.  
		for (n = 1; n < 4; n++)
		{
			t = 0;
			for (m = 1; m <= n - 1; m++) t += LU[n][m] * Y[m];
			Y[n] = B[i][P[n]] - t;
		}

		//Solving Ux = y.  
		for (n = 4 - 1; n >= 1; n--)
		{
			t = 0;
			for (m = n + 1; m < 4; m++) t += LU[n][m] * X[m];
			X[n] = (Y[n] - t) / LU[n][n];
		}//Now, X contains the solution.  

		for (j = 1; j < 4; j++) B[i][j] = X[j]; //Copying 'X' into the same row of 'B'.  
	 }//Now, 'B' the transpose of the inverse of 'A'.  

	  /* Copying transpose of 'B' into 'LU', which would the inverse of 'A'. */
	for (i = 1; i < 4; i++) for (j = 1; j < 4; j++) LU[i][j] = B[j][i];

	return 0;

}
