////////////////////////////////////////////////////////////////////////////////
// File: gauss_elimination.c                                                  //
// Routines:                                                                  //
//    Gaussian_Elimination                                                    //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  int Gaussian_Elimination(double *A, int n, double *B)                     //
//                                                                            //
//     Solve the linear system of equations AX=B where A is an n x n matrix   //
//     B is an n-dimensional column vector (n x 1 matrix) for the             //
//     n-dimensional column vector (n x 1 matrix) X.                          //
//                                                                            //
//     This routine performs partial pivoting and the elements of A are       //
//     modified during computation.  The result X is returned in B.           //
//     If the matrix A is singular, the return value of the function call is  //
//     -1. If the solution was found, the function return value is 0.         //
//                                                                            //
//  Arguments:                                                                //
//     double *A      On input, the pointer to the first element of the       //
//                    matrix A[n][n].  On output, the matrix A is destroyed.  //
//     int     n      The number of rows and columns of the matrix A and the  //
//                    dimension of B.                                         //
//     double *B      On input, the pointer to the first element of the       //
//                    vector B[n].  On output, the vector B is replaced by the//
//                    vector X, the solution of AX = B.                       //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], B[N];                                                  //
//                                                                            //
//     (your code to create the matrix A and vector B )                       //
//     err = Gaussian_Elimination((double*)A, NROWS, B);                      //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else { printf(" The Solution is: \n"); ...                             //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
#include <math.h>                                     // required for fabs()

int Gaussian_Elimination(double *A, int n, double *B)
{
   int row, i, j, pivot_row;
   double max, dum, *pa, *pA, *A_pivot_row;

      // for each variable find pivot row and perform forward substitution

   pa = A;
   for (row = 0; row < (n - 1); row++, pa += n) {

                       //  find the pivot row

      A_pivot_row = pa;
      max = fabs(*(pa + row));
      pA = pa + n;
      pivot_row = row;
      for (i = row + 1; i < n; pA += n, i++)
         if ((dum = fabs(*(pA + row))) > max) { 
            max = dum; A_pivot_row = pA; pivot_row = i; 
         }
      if (max == 0.0) return -1;                // the matrix A is singular

        // and if it differs from the current row, interchange the two rows.
             
      if (pivot_row != row) {
         for (i = row; i < n; i++) {
            dum = *(pa + i);
            *(pa + i) = *(A_pivot_row + i);
            *(A_pivot_row + i) = dum;
         }
         dum = B[row];
         B[row] = B[pivot_row];
         B[pivot_row] = dum;
      }

                      // Perform forward substitution

      for (i = row + 1; i < n; i++) {
         pA = A + i * n;
         dum = - *(pA + row) / *(pa + row);
         *(pA + row) = 0.0;
         for (j = row + 1; j < n; j++) *(pA + j) += dum * *(pa + j);
         B[i] += dum * B[row];
      }
   }

                    // Perform backward substitution
  
   pa = A + (n - 1) * n;
   for (row = n - 1; row >= 0; pa -= n, row--) {
      if ( *(pa + row) == 0.0 ) return -1;           // matrix is singular
      dum = 1.0 / *(pa + row);
      for ( i = row + 1; i < n; i++) *(pa + i) *= dum; 
      B[row] *= dum; 
      for ( i = 0, pA = A; i < row; pA += n, i++) {
         dum = *(pA + row);
         for ( j = row + 1; j < n; j++) *(pA + j) -= dum * *(pa + j);
         B[i] -= dum * B[row];
      }
   }
   return 0;
}
