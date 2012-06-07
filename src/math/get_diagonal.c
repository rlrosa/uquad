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
// File: get_diagonal.c                                                       //
// Routine(s):                                                                //
//    Get_Diagonal                                                            //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Get_Diagonal(double v[], double *A, int nrows, int ncols)            //
//                                                                            //
//  Description:                                                              //
//     Copy the diagonal A[i][i], i = 0, ..., min( nrows, ncols ) to the      //
//     vector v.                                                              //
//     Note that v should be declared "double v[N]", with                     //
//     N >= min(nrows, ncols) in the calling routine.                         //
//                                                                            //
//  Arguments:                                                                //
//     double v[]   Destination address of the diagonal of the matrix A.      //
//     double *A    Pointer to the first element of the source matrix A.      //
//     int    nrows The number of rows matrix A.                              //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N],  v[N];                                                 //
//                                                                            //
//     (your code to set the matrix A )                                       //
//                                                                            //
//     Get_Diagonal(v, &A[0][0],  M, N);                                      //
//     printf("The diagonal is \n"); ... }                                    //
////////////////////////////////////////////////////////////////////////////////
void Get_Diagonal(double v[], double *A, int nrows, int ncols)
{
   int i, n;

   n = (nrows < ncols) ? nrows: ncols;
   for (i = 0; i < n; A += (ncols + 1), i++) v[i] = *A;
}
