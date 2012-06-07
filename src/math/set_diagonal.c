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
// File: set_diagonal.c                                                       //
// Routine(s):                                                                //
//    Set_Diagonal                                                            //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Set_Diagonal(double *A, double v[], int nrows, int ncols)            //
//                                                                            //
//  Description:                                                              //
//     Copy the vector v to the diagonal A[i][i], where 0 <= i <              //
//     min( nrows, ncols ).                                                   //
//     Note that v should be declared "double v[N]", N >= min( nrows, ncols ) //
//     in the calling routine.                                                //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the source matrix A.      //
//     double v[]   Source of the new diagonal for the matrix A.              //
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
//     (your code to initialize the matrix A and the vector v)                //
//                                                                            //
//     Set_Diagonal(&A[0][0], v, M, N);                                       //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Set_Diagonal(double *A, double v[], int nrows, int ncols)
{
   int n;

   if (nrows < ncols) n = nrows; else n = ncols;

   for (; n > 0 ; A += (ncols + 1), n--)  *A = *v++;
}
