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
// File: hessenberg_orthog.c                                                  //
// Routine(s):                                                                //
//    Hessenberg_Form_Orthogonal                                              //
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>                      // required for malloc() and free()    
#include <math.h>                        // required for sqrt()

//                    Required Externally Defined Routines 
void Identity_Matrix(double *A, int n);

////////////////////////////////////////////////////////////////////////////////
//  int Hessenberg_Form_Orthogonal(double *A, double *U, int n)               //
//                                                                            //
//  Description:                                                              //
//     This program transforms the square matrix A to a similar matrix in     //
//     Hessenberg form by a multiplying A on the right and left by a sequence //
//     of Householder transformations.                                        //
//     Def:  Two matrices A and B are said to be orthogonally similar if there//
//           exists an orthogonal matrix U such that A U = U B.               //
//     Def   A Hessenberg matrix is the sum of an upper triangular matrix and //
//           a matrix all of whose components are 0 except possibly on its    //
//           subdiagonal.  A Hessenberg matrix is sometimes said to be almost //
//           upper triangular.                                                //
//     Def:  A Householder transformation is an orthogonal transformation of  //
//           the form Q = I - 2 uu'/u'u, where u is a n x 1 column matrix and //
//           ' denotes the transpose.                                         //
//     Thm:  If Q is a Householder transformation then Q' = Q  and  Q' Q = I, //
//           i.e. Q is a symmetric involution.                                //
//     The algorithm proceeds by successivly selecting columns j = 0,...,n-3  //
//     and then calculating the Householder transformation Q which annihilates//
//     the components below the subdiagonal for that column and leaves the    //
//     previously selected columns invariant.  The algorithm then updates     //
//     the matrix A, in place, by premultiplication by Q followed by          //
//     postmultiplication by Q.                                               //
//     If the j-th column of A is (a[0],...,a[n-1]), then  choose u' =        //
//     (u[0],...,u[n-1]) by u[0] = 0, ... , u[j] = 0, u[j+2] = a[j+2],...,    //
//     u[n-1] = a[n-1].  The remaining component u[j+1] = a[j+1] - s, where   //
//     s^2 = a[j+1]^2 + ... + a[n-1]^2, and the choice of sign for s,         //
//     sign(s) = -sign(a[j+1]) maximizes the number of significant bits for   //
//     u[j+1].                                                                //
//                                                                            //
//     Remark:  If H = U' A U, where U is orthogonal, and if v is an eigen-   //
//     vector of H with eigenvalue x, then A U v = U H v = x U v, so that     //
//     U v is an eigenvector of A with corresponding eigenvalue x.            //
//                                                                            //
//  Arguments:                                                                //
//     double *A   Pointer to the first element of the matrix A[n][n].        //
//                 The original matrix A is replaced with the orthogonally    //
//                 similar matrix in Hessenberg form.                         //
//     double *U   Pointer to the first element of the matrix U[n][n].  The   //
//                 orthogonal matrix which transforms the input matrix to     //
//                 an orthogonally similar matrix in Hessenberg form.         //
//     int     n   The number of rows or columns of the matrix A.             //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - Unable to allocate space for working storage.             //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], U[N][N];                                               //
//                                                                            //
//     (your code to create the matrix A)                                     //
//     Hessenberg_Form_Orthogonal(&A[0][0], (double*) U, N);                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Hessenberg_Form_Orthogonal(double *A, double *U, int n)
{
   int i, k, col;
   double *u;
   double *p_row, *psubdiag;
   double *pA, *pU;
   double sss;                             // signed sqrt of sum of squares
   double scale;
   double innerproduct;

         // n x n matrices for which n <= 2 are already in Hessenberg form

   Identity_Matrix(U, n);
   if (n <= 2) return 0;

         // Reserve auxillary storage, if unavailable, return an error

   u = (double *) malloc(n * sizeof(double));
   if (u == NULL) return -1;

           // For each column use a Householder transformation 
           //   to zero all entries below the subdiagonal.

   for (psubdiag = A + n, col = 0; col < (n - 2); psubdiag += (n+1), col++) {

          // Calculate the signed square root of the sum of squares of the
          // elements below the diagonal.

      for (pA = psubdiag, sss = 0.0, i = col + 1; i < n; pA += n, i++)
         sss += *pA * *pA;
      if (sss == 0.0) continue;
      sss = sqrt(sss);
      if ( *psubdiag >= 0.0 ) sss = -sss;

          // Calculate the Householder transformation Q = I - 2uu'/u'u.

      u[col + 1] = *psubdiag - sss;
      *psubdiag = sss; 
      for (pA = psubdiag + n, i = col + 2; i < n; pA += n, i++) {
         u[i] = *pA;
         *pA = 0.0;
      }

                    // Premultiply A by Q
   
      scale = -1.0 / (sss * u[col+1]);
      for (p_row = psubdiag - col, i = col + 1; i < n; i++) {
         pA = A + n * (col + 1) + i;
         for (innerproduct = 0.0, k = col + 1; k < n; pA += n, k++) 
            innerproduct += u[k] * *pA;
         innerproduct *= scale;
         for (pA = p_row + i, k = col + 1; k < n; pA += n, k++) 
            *pA -= u[k] * innerproduct;
      }
        
                    // Postmultiply QA by Q

      for (p_row = A, i = 0; i < n; p_row += n, i++) {
         for (innerproduct = 0.0, k = col + 1; k < n; k++) 
            innerproduct += u[k] * *(p_row + k);
         innerproduct *= scale;
         for (k = col + 1; k < n; k++) 
            *(p_row + k) -= u[k] * innerproduct;
      }
        
                    // Postmultiply U by (I - 2uu')

      for (i = 0, pU = U; i < n; pU += n, i++) {
         for (innerproduct = 0.0, k = col + 1; k < n; k++) 
            innerproduct += u[k] * *(pU + k);
         innerproduct *= scale;
         for (k = col + 1; k < n; k++) 
            *(pU + k) -= u[k] * innerproduct;
      }

   }

   free(u);

   return 0;
}
