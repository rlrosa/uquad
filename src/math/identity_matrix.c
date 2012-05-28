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
// File: identity_matrix.c                                                    //
// Routine(s):                                                                //
//    Identity_Matrix                                                         //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Identity_Matrix(double *A, int n)                                    //
//                                                                            //
//  Description:                                                              //
//     Set the square n x n matrix A equal to the identity matrix, i.e.       //
//     A[i][j] = 0 if i != j and A[i][i] = 1.                                 //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    n     The number of rows and columns of the matrix A.           //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N];                                                        //
//                                                                            //
//     Identity_Matrix(&A[0][0], N);                                          //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Identity_Matrix(double *A, int n)
{
   int i,j;

   for (i = 0; i < n - 1; i++) {
      *A++ = 1.0;
      for (j = 0; j < n; j++) *A++ = 0.0;
   } 
   *A = 1.0;
}
