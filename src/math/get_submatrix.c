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
// File: get_submatrix.c                                                      //
// Routine(s):                                                                //
//    Get_Submatrix                                                           //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Get_Submatrix(double *S, int mrows, int mcols,                       //
//                                   double *A, int ncols, int row, int col)  //
//                                                                            //
//  Description:                                                              //
//     Copy the mrows and mcols of the nrows x ncols matrix A starting with   //
//     A[row][col] to the submatrix S.                                        //
//     Note that S should be declared double S[mrows][mcols] in the calling   //
//     routine.                                                               //
//                                                                            //
//  Arguments:                                                                //
//     double *S    Destination address of the submatrix.                     //
//     int    mrows The number of rows of the matrix S.                       //
//     int    mcols The number of columns of the matrix S.                    //
//     double *A    Pointer to the first element of the matrix A[nrows][ncols]//
//     int    ncols The number of columns of the matrix A.                    //
//     int    row   The row of A corresponding to the first row of S.         //
//     int    col   The column of A corresponding to the first column of S.   //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     #define NB                                                             //
//     #define MB                                                             //
//     double A[M][N],  B[MB][NB];                                            //
//     int row, col;                                                          //
//                                                                            //
//     (your code to set the matrix A, the row number row and column number   //
//      col)                                                                  //
//                                                                            //
//     if ( (row >= 0) && (col >= 0) && ((row + MB) < M) && ((col + NB) < N) )//
//        Get_Submatrix(&B[0][0], MB, NB, &A[0][0], N, row, col);             //
//     printf("The submatrix B is \n"); ... }                                 //
////////////////////////////////////////////////////////////////////////////////

#include <string.h>                                    // required for memcpy()

void Get_Submatrix(double *S, int mrows, int mcols, 
                                        double *A, int ncols, int row, int col)
{
   int number_of_bytes = sizeof(double) * mcols;

   for (A += row * ncols + col; mrows > 0; A += ncols, S+= mcols, mrows--) 
      memcpy(S, A, number_of_bytes);
}
