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
// File: subtract_matrices.c                                                  //
// Routine(s):                                                                //
//    Subtract_Matrices                                                       //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Subtract_Matrices(double *C, double *A, double *B, int nrows,        //
//                                                               int ncols)   //
//                                                                            //
//  Description:                                                              //
//     Subtract the matrix B from the matrix A to form the matrix C, i.e.     //
//     C = A - B.                                                             //
//     All matrices should be declared as " double X[nrows][ncols] " in the   //
//     calling routine, where X = A, B, C.                                    //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Pointer to the first element of the matrix C.             //
//     double *A    Pointer to the first element of the matrix A.             //
//     double *B    Pointer to the first element of the matrix B.             //
//     int    nrows The number of rows of matrices A and B.                   //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N],  B[M][N], C[M][N];                                     //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Subtract_Matrices(&C[0][0], &A[0][0], &B[0][0], M, N);                 //
//     printf("The matrix C = A - B  is \n"); ...                             //
////////////////////////////////////////////////////////////////////////////////
void Subtract_Matrices(double *C, double *A, double *B, int nrows,
                                                                    int ncols) 
{
   register int i;
   register int n = nrows * ncols;

   for (i = 0; i < n; i++) C[i] = A[i] - B[i];
}
