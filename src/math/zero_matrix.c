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
// File: zero_matrix.c                                                        //
// Routine(s):                                                                //
//    Zero_Matrix                                                             //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Zero_Matrix(double *A, int nrows, int ncols)                         //
//                                                                            //
//  Description:                                                              //
//     Set the nrows x ncols matrix A equal to the zero matrix, i.e.          //
//     A[i][j] = 0 for all i, j.                                              //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of the matrix A.                       //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[N][M];                                                        //
//                                                                            //
//     Zero_Matrix(&A[0][0], N, M);                                           //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////

void Zero_Matrix(double *A, int nrows, int ncols)
{
   int n = nrows * ncols;

   for (; n > 0; n--) *A++ = 0.0;
}
