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
// File: set_submatrix.c                                                      //
// Routine(s):                                                                //
//    Set_Submatrix                                                           //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Set_Submatrix(double *A, int ncols, double *S, int mrows, int mcols, //
//                                                        int row, int col)   //
//                                                                            //
//  Description:                                                              //
//     Copy the mrows x mcols submatrix S into the nrows x ncols matrix A     //
//     starting at the location A[row][col].                                  //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A[n][n].       //
//     int    ncols The number of columns of the matrix A.                    //
//     double *S    Destination address of the submatrix.                     //
//     int    mrows The number of rows matrix S.                              //
//     int    mcols The number of columns of the matrix S.                    //
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
//     (your code to initialize the matrix A, submatrix B, row number row,    //
//      and column number col )                                               //
//                                                                            //
//     if ( (row > 0) && ( row + MB < M ) && ( col > 0 ) && (col + NB < N)    //
//        Set_Submatrix(&A[0][0], N, &B[0][0], MB, NB, row, col);             //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Set_Submatrix(double *A, int ncols, double *S, int mrows, int mcols,
                                                           int row, int col) {
   int i,j;

   for ( A += row * ncols + col, i = 0; i < mrows; A += ncols, i++) 
      for (j = 0; j < mcols; j++) *(A + j) = *S++;
}
