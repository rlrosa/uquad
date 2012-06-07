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
// File: join_by_rows.c                                                       //
// Routine(s):                                                                //
//    Join_Matrices_by_Row                                                    //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Join_Matrices_by_Row(double *C, double *A, int nrows, int ncols,     //
//                                                     double *B, int mcols)  //
//                                                                            //
//  Description:                                                              //
//     Copy the nrows x ncols matrix A into the nrows x (ncols + mcols)       //
//     matrix C and then copy the nrows x mcols matrix B to starting at       //
//     the location C[0][ncols], i.e. C = [A:B].                              //
//     The matrix C should be declared as double C[nrows][ncols + mcols] in   //
//     the calling routine.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Pointer to the first element of the matrix C.             //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of matrices A and B.                   //
//     int    ncols The number of columns of the matrix A.                    //
//     double *B    Pointer to the first element of the matrix B.             //
//     int    mcols The number of columns of the matrix B.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     #define NB                                                             //
//     double A[M][N],  B[M][NB], C[M][N+NB];                                 //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Join_Matrices_by_Row(&C[0][0], &A[0][0], M, N, &B[0][0], NB);          //
//     printf("The matrix C is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////

#include <string.h>                             // required for memcpy()

void Join_Matrices_by_Row(double *C, double *A, int nrows, int ncols, 
                                                          double *B, int mcols) 
{
   for (; nrows > 0; nrows--) { 
      memcpy( C, A, sizeof(double) * ncols );
      C += ncols;
      A += ncols;
      memcpy( C, B, sizeof(double) * mcols );
      C += mcols;
      B += mcols;
   }
}
