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
// File: multiply_matrices_3x3.c                                              //
// Routine(s):                                                                //
//    Multiply_Matrices_3x3                                                   //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Multiply_Matrices_3x3(double *C, double *A, double *B)               //
//                                                                            //
//  Description:                                                              //
//     Post multiply the 3 x 3 matrix A by the 3 x 3 matrix B to form the     //
//     3 x 3 matrix C, i.e. C = A B.                                          //
//     All matrices should be declared as double X[3][3] in the calling       //
//     routine where X = A, B, C.                                             //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Pointer to the first element of the matrix C.             //
//     double *A    Pointer to the first element of the matrix A.             //
//     double *B    Pointer to the first element of the matrix B.             //
//                                                                            //
//  Return Values:                                                            //
//     Pointer to C.                                                          //
//                                                                            //
//  Example:                                                                  //
//     double A[3][3],  B[3][3], C[3][3];                                     //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Multiply_Matrices_3x3((double *)C, &A[0][0], &B[0][0]);                //
//     printf("The matrix C is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Multiply_Matrices_3x3(double *C, double *A, double *B) 
{
   C[0] = A[0] * B[0] + A[1] * B[3] + A[2]*B[6];
   C[1] = A[0] * B[1] + A[1] * B[4] + A[2]*B[7];
   C[2] = A[0] * B[2] + A[1] * B[5] + A[2]*B[8];
   C[3] = A[3] * B[0] + A[4] * B[3] + A[5]*B[6];
   C[4] = A[3] * B[1] + A[4] * B[4] + A[5]*B[7];
   C[5] = A[3] * B[2] + A[4] * B[5] + A[5]*B[8];
   C[6] = A[6] * B[0] + A[7] * B[3] + A[8]*B[6];
   C[7] = A[6] * B[1] + A[7] * B[4] + A[8]*B[7];
   C[8] = A[6] * B[2] + A[7] * B[5] + A[8]*B[8];
}
