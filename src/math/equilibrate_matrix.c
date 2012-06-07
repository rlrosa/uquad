/**
 * Copyright (C) 2004 RLH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
////////////////////////////////////////////////////////////////////////////////
// File: equilibrate_matrix.c                                                 //
// Routines:                                                                  //
//    Equilibrate_Matrix                                                      //
//    Equilibrate_Right_Hand_Side                                             //
//    Unequilibrate_Solution                                                  //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  Description:                                                              //
//     The purpose for equilibration of a matrix is to transform the matrix   //
//     to another matrix with a lower condition number by scaling the         //
//     the rows and columns of the original matrix.                           //
//                                                                            //
//     Given a system of linear equations  Ax = B, where A is an nxn square   //
//     matrix and x and B are vectors (nx1 matrices), the matrix A is         //
//     equilibrated by finding real diagonal matrices R and C such that the   //
//     condition number of RAC is less than that of the matrix A.  There are  //
//     infinitely many possible algorithms for the row scaling factors and    //
//     column scaling factors, but the most common methods are:               //
//        (1)  Set r[i] = 1 / max(|a[i][j]| : j = 0,...,n-1), i=0,...,n-1.    //
//             Set c[j] = 1 / max(|r[i]a[i][j]| : i = 0,...,n-1), j=0,..,n-1. //
//        (2)  Set r[i] = 1 / Sum |a[i][j]|, where the sum is over j,         //
//                                                               i=0,...,n-1. //
//             Set c[j] = 1.0, j=0,...,n-1                                    //
//        (3)  A variation of method (1) in which                             //
//             r[i] = 1/2^k, k = max( m : |a[i][j]| = x 2^m, 1/2 < x <= 1,    //
//                                              j = 0,...,n-1), i=0,...,n-1.  //
//             c[j] = 1/2^k, k = max( m : |r[i]a[i][j]| = x 2^m, 1/2 < x <= 1,//
//                                              i = 0,...,n-1), j=0,...,n-1.  //
//             By choosing the scaling factors as powers of 2, there is no    //
//             loss in the number of significant bits of the scaled matrix    //
//             elements.  Note that if a[i][j] = 0, then m is not defined.    //
//             If the i-th row consists entirely of 0's, then r[i] is set to  //
//             1.0 and if the j-th column consists entirely of 0's, then c[j] //
//             is set to 1.0.                                                 //
//                                                                            //
//     The equilibrated system, RACy = RB, is then solved for y using a       //
//     linear equation solver.  The solution of the original system of linear //
//     equations, the unequilibrated system, is x = Cy.  The advantage of     //
//     method (2) is that C is the identity so x = y.                         //
//                                                                            //
//     In the routine below, Equilibrate_Matrix(), method (3) is used to      //
//     equilibrate the matrix A where the diagonal scaling factors are        //
//     returned in the array r[] and the column scaling factors are returned  //
//     in the array c[].  The routine, Equilibrate_Matrix(), returns 0 if     //
//     equilibration of the matrix A is successful, bit 0 is set if           //
//     equilibration results in an underflow, i.e. a loss of significance of  //
//     a matrix element of RAC.  The matrix A is transformed as A -> RAC,     //
//     with a possible underflow if the return bit 0 is set.                  //
//                                                                            //
//     The routine, Equilibrate_Right_Hand_Side(), the diagonal elements of   //
//     or the row scaling elements are used to transform the right-hand side  //
//     of the linear equation Ax=B. Equilibrate_Right_Hand_Side() returns a 0 //
//     if multiplication of R by the vector B is successful, bit 0 is set if  //
//     multiplication of R by B results in an underflow, or bit 1 is set if   //
//     multiplication of R by B results in an overflow.  If an overflow is    //
//     detected, then the multiplication process is terminated immediately.   //
//     The complex vector B is transformed as B -> RB.  In case of an         //
//     overflow, the vector B was modified but not completely transformed.    //
//                                                                            //
//     If the matrix A is successfully equilibrated and the vector B is       //
//     successfully transformed, then a linear equation solver can be called  //
//     to solve the equilibrated system RACy = RB for y.                      //
//                                                                            //
//     The routine, Unequilibrate_Solution(), is then used to transform the   //
//     solution of the equilibrated system to a solution of the original      //
//     unequilibrated system of linear equations.  Unequilibrate_Solution()   //
//     returns a 0 if multiplication of C by the solution vector of the       //
//     equilibrated system, y, is successful, bit 0 is set if multiplication  //
//     of C by y results in an underflow, or bit 1 is set if multiplication   //
//     of C by y results in an overflow.  If an overflow is detected, then the//
//     multiplication process is terminated immediately, otherwise the vector //
//     y is transformed as y -> Cy.                                           //
////////////////////////////////////////////////////////////////////////////////

#include <float.h>             // required for DBL_MIN_EXP and DBL_MAX_EXP
#include <math.h>              // required for frexp() and ldexp()  
#define SUCCESS ERROR_OK
#define UNDERFLOWS ERROR_MATH_UNDERFLOWS
#define OVERFLOWS ERROR_MATH_OVERFLOWS

////////////////////////////////////////////////////////////////////////////////
// int Equilibrate_Matrix(double *A, int nrows, int ncols, double r[],        //
//                                                                double c[]) //
//                                                                            //
//  Description:                                                              //
//     This routine is called to equilibrate the elements of the possibly     //
//     ill-conditioned nrows x ncols matrix A.  Note that for solving a system//
//     of linear equations, the matrix A is a square matrix so that the number//
//     of rows and the number of columns are equal.                           //
//                                                                            //
//     The matrix A is equilibrated by finding diagonal matrices R and C such //
//     that the absolute value of each element of the equilibrated matrix RAC //
//     is bounded by 1.  By choosing the scaling factors as powers of 2, there//
//     is no loss in the number of significant bits of the scaled matrix      //
//     elements.                                                              //
//                                                                            //
//     In the routine below, Equilibrate_Matrix(), the diagonal elements of   //
//     the column scaling factors are returned in the array c[] while the     //
//     the diagonal elements of the row scaling factors are returned in the   //
//     in the array r[].  The routine, Equilibrate_Matrix(), returns a 0 if   //
//     equilibration of the matrix A is successful, bit 0 is set if           //
//     equilibration results in an underflow, i.e. a loss of significance of  //
//     a matrix element of A.  The matrix A is transformed as A -> RAC, with  //
//     a possible underflow if the return bit 0 is set.                       //
//                                                                            //
//  Arguments:                                                                //
//    double *A                                                               //
//       The address of the first element of the matrix A.                    //
//       Declared in the calling routine as double A[nrows][ncols].           //
//       On return the matrix A contains the equilibrated matrix.             //
//    int    nrows                                                            //
//       The number of rows of the matrix A.                                  //
//    int    ncols                                                            //
//       The number of columns of the matrix A.                               //
//    double r[]                                                              //
//       On return, the array containing the row scale factors.               //
//       Declared in the calling routine as r[nrows].                         //
//    double c[]                                                              //
//       On return, the array containing the column scale factors.            //
//       Declared in the calling routine as c[ncols].                         //
//                                                                            //
//    Return Values:                                                          //
//        0 Success                                                           //
//        1 Scaling results in an underflow, i.e. loss of significance.       //
//                                                                            //
//        The matrix A is transformed as A -> RAC, with a possible underflow  //
//        if the return bit 0 is set.                                         //
//                                                                            //
//        The array r[] is set with the diagonal of the diagonal matrix R,    //
//        and the array c[] is set with the diagonal of the diagonal matrix C.//
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[N][M], r[N], c[M];                                            //
//     int return_value;                                                      //
//                                                                            //
//     (your code to initialize the matrix A )                                //
//                                                                            //
//     return_value = Equilibrate_Matrix(&A[0][0], N, M, r, c);               //
//     ...                                                                    //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Equilibrate_Matrix(double *A, int nrows, int ncols, double r[], double c[])
{
   double *pA, *pC;
   double x;
   int min, max, exponent;
   int nonzero;
   int i,j;//,k;//unused
   int return_code = SUCCESS;

                          // Find row scale factors.
   
   for (i = 0, pA = A; i < nrows; i++, pA += ncols) {
      max = DBL_MIN_EXP;
      min = DBL_MAX_EXP;
      nonzero = 0;
      r[i] = 1.0;
      for (j = 0, pC = pA; j < ncols; j++, pC++) {
         x = frexp(*pC, &exponent);
         if ( x == 0.0 ) continue;
         if (exponent > max) max = exponent;
         if (exponent < min) min = exponent;
         nonzero = 1;
      }
      if (nonzero) { 
         if ( min - max < DBL_MIN_EXP ) return_code |= UNDERFLOWS;
         r[i] = ldexp(1.0, -max);
         for (j = 0, pC = pA; j < ncols; j++, pC++) *pC *= r[i];
      }
   }

                         // Find Column Scale Factors.

   for (i = 0, pA = A; i < ncols; i++, pA++) {
      max = DBL_MIN_EXP;
      min = DBL_MAX_EXP;
      nonzero = 0;
      c[i] = 1.0;
      for (j = 0, pC = pA; j < nrows; j++, pC += ncols) {
         x = frexp(*pC, &exponent);
         if ( x == 0.0 ) continue;
         if (exponent > max) max = exponent;
         if (exponent < min) min = exponent;
         nonzero = 1;
      }
      if (nonzero) {
         if ( min - max < DBL_MIN_EXP ) return_code = UNDERFLOWS;
         c[i] = ldexp(1.0, -max);
         for (j = 0, pC = pA; j < nrows; j++, pC += ncols) *pC *= c[i];
      }
   }
  
   return return_code;
}


////////////////////////////////////////////////////////////////////////////////
//  int Equilibrate_Right_Hand_Side(double B[], double r[], int n)            //
//                                                                            //
//  Description:                                                              //
//     This routine multiplies each element of B[] by the corresponding       //
//     element of r[].  I.e. B[i] -> B[i]*r[i], for i = 0,...,n-1.            //
//                                                                            //
//  Arguments:                                                                //
//    double B[]                                                              //
//       The address of the first element of the vector B.                    //
//       Declared in the calling routine as double B[n].                      //
//       On return, B is transformed to RB.                                   //
//    double r[]                                                              //
//       The address of the first element of the vector r which was calculated//
//       by calling Equilibrate_Matrix().                                     //
//       Declared in the calling routine as double r[n].                      //
//    int    n                                                                //
//       The number of elements in the vectors B and r.                       //
//                                                                            //
//    Return Values:                                                          //
//        0 Success                                                           //
//        1 Scaling results in an underflow, i.e. loss of significance.       //
//        2 Scaling results in an overflow, process halted.                   //
//        3 Scaling results in both an underflow and an overflow, process     //
//          halted.                                                           //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], B[N], r[N], c[N];                                      //
//                                                                            //
//     (your code to initialize the vector B and matrix A)                    //
//                                                                            //
//     return_code = Equilibrate_Matrix(&A[0][0], N, N, r, c);                //
//     printf("return codes for equilibrate matrix\n");                       //
//     if (return_code == 0) printf("success\n");                             //
//     if (return_code &= 1) printf("warning loss of significance\n");        //
//     return_code = Equilibrate_Right_Hand_Side(B, r, N);                    //
//     printf("return codes for equilibrate right hand side\n");              //
//     if (return_code == 0) printf("success\n");                             //
//     if (return_code &= 1) printf("warning loss of significance\n");        //
//     if (return_code &= 2) printf("fatal error - overflow \n");             //
//     ...                                                                    //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Equilibrate_Right_Hand_Side(double B[], double r[], int n) 
{
   int i;
   int ret_code = SUCCESS;
   int B_exponent, r_exponent;
   double rx, Bx;

   for (i = 0; i < n; i++) {
      Bx = frexp(B[i], &B_exponent);
      rx = frexp(r[i], &r_exponent);
      if (r_exponent + B_exponent < DBL_MIN_EXP) ret_code |= UNDERFLOWS;
      if (r_exponent + B_exponent > DBL_MAX_EXP) return ret_code += OVERFLOWS;
      B[i] *= r[i];
   }

   return ret_code;
}


////////////////////////////////////////////////////////////////////////////////
//  int Unequilibrate_Solution(double x[], double c[], int n)                 //
//                                                                            //
//  Description:                                                              //
//     This routine multiplies each element of x[] by the corresponding       //
//     element of c[].  I.e. x[i] -> x[i]*c[i], for i = 0,...,n-1.            //
//                                                                            //
//  Arguments:                                                                //
//    double *x                                                               //
//       The address of the first element of the vector x.                    //
//       Declared in the calling routine as double complex x[n].              //
//       On input, x is the solution of the equilibrated system of linear     //
//       equations, and on output, x is the solution of the unequilibrated    //
//       original system of linear equations provided the return code is 0.   //
//    double c[]                                                              //
//       The address of the first element of the vector c which was calculated//
//       by calling Equilibrate_Matrix().                                     //
//       Declared in the calling routine as double c[n].                      //
//    int    n                                                                //
//       The number of elements in the vectors x and c.                       //
//                                                                            //
//    Return Values:                                                          //
//        0 Success                                                           //
//        1 Scaling results in an underflow, i.e. loss of significance.       //
//        2 Backscaling results in an overflow, process halted.               //
//        3 Backscaling results both an underflow and an overflow, process    //
//          halted.                                                           //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double x[N];                                                           //
//     double A[N][N], B[N], r[N], c[N];                                      //
//                                                                            //
//     (your code to initialize the vector B, and matrix A)                   //
//                                                                            //
//     return_code = Equilibrate_Matrix(&A[0][0], N, N, r, c);                //
//     printf("return codes for equilibrate matrix\n");                       //
//     if (return_code == 0) printf("success\n");                             //
//     if (return_code &= 1) printf("warning loss of significance\n");        //
//     return_code = Equilibrate_Right_Hand_Side(B, r, N);                    //
//     printf("return codes for equilibrate right hand side\n");              //
//     if (return_code == 0) printf("success\n");                             //
//     if (return_code &= 1) printf("warning loss of significance\n");        //
//     if (return_code &= 2) printf("fatal error - overflow \n");             //
//     if (return_code &= 2) exit(0);                                         //
//     (Call linear equation solver, return solution in x.)                   //
//     (If return from equation solver fails then exit)                       //
//     return_code = Unequilibrate_Solution(x, c, N);                         //
//     printf("return codes for unequilibrate solution\n");                   //
//     if (return_code == 0) printf("success\n");                             //
//     if (return_code &= 1) printf("warning loss of significance\n");        //
//     if (return_code &= 2) printf("fatal error - overflow \n");             //
//     ...                                                                    //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Unequilibrate_Solution(double x[], double c[], int n) 
{
   int i;
   int ret_code = SUCCESS;
   int c_exponent, x_exponent;
   double xx, cx;

   for (i = 0; i < n; i++) {
      xx = frexp(x[i], &x_exponent);
      cx = frexp(c[i], &c_exponent);
      if (c_exponent + x_exponent < DBL_MIN_EXP) ret_code = UNDERFLOWS;
      if (c_exponent + x_exponent > DBL_MAX_EXP) return ret_code += OVERFLOWS;
      x[i] *= c[i];
   }

   return ret_code;
};
