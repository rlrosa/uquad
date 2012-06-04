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
// File: multiply_matrices.c                                                  //
// Routine(s):                                                                //
//    Multiply_Matrices                                                       //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Multiply_Matrices(double *C, double *A, int nrows, int ncols,        //
//                                                    double *B, int mcols)   //
//                                                                            //
//  Description:                                                              //
//     Post multiply the nrows x ncols matrix A by the ncols x mcols matrix   //
//     B to form the nrows x mcols matrix C, i.e. C = A B.                    //
//     The matrix C should be declared as double C[nrows][mcols] in the       //
//     calling routine.  The memory allocated to C should not include any     //
//     memory allocated to A or B.                                            //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Pointer to the first element of the matrix C.             //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of the matrices A and C.               //
//     int    ncols The number of columns of the matrices A and the           //
//                   number of rows of the matrix B.                          //
//     double *B    Pointer to the first element of the matrix B.             //
//     int    mcols The number of columns of the matrices B and C.            //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     #define NB                                                             //
//     double A[M][N],  B[N][NB], C[M][NB];                                   //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Multiply_Matrices(&C[0][0], &A[0][0], M, N, &B[0][0], NB);             //
//     printf("The matrix C is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Multiply_Matrices(double *C, double *A, int nrows, int ncols,
                                                          double *B, int mcols) 
{
   double *pA = A;
   double *pB;
   double *p_B;
   //   double *pC = C;//unused
   int i,j,k;

   for (i = 0; i < nrows; A += ncols, i++) 
      for (p_B = B, j = 0; j < mcols; C++, p_B++, j++) {
         pB = p_B;
         pA = A;
         *C = 0.0; 
         for (k = 0; k < ncols; pA++, pB += mcols, k++) 
            *C += *pA * *pB;
      }
}
