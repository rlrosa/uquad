////////////////////////////////////////////////////////////////////////////////
// File: join_by_cols.c                                                       //
// Routine(s):                                                                //
//    Join_Matrices_by_Column                                                 //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Join_Matrices_by_Column(double *C, double *A, int nrows, int ncols,  //
//                                                    double *B, int mrows)   //
//                                                                            //
//  Description:                                                              //
//     Copy the nrows x ncols matrix A into the (nrows + mrows) x ncols       //
//     matrix C and then copy the mrows x ncols matrix B to starting at       //
//     the location C[nrows][0], i.e. C' = [A':B']', where ' denotes the      //
//     transpose.                                                             //
//     The matrix C should be declared as double C[nrows + mrows][ncols] in   //
//     the calling routine.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Pointer to the first element of the matrix C.             //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of matrix A.                           //
//     int    ncols The number of columns of the matrices A and B.            //
//     double *B    Pointer to the first element of the matrix B.             //
//     int    mrows The number of rows of the matrix B.                       //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     #define MB                                                             //
//     double A[M][N],  B[MB][N], C[M+MB][N];                                 //
//     int row, col;                                                          //
//                                                                            //
//     (your code to set the matrices A and B)                                //
//                                                                            //
//     Join_Matrices_by_Column(&C[0][0], &A[0][0], M, N, &B[0][0], MB);       //
//     printf("The matrix C is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////

#include <string.h>                                // required for memcpy()

void Join_Matrices_by_Column(double *C, double *A, int nrows, int ncols, 
                                                          double *B, int mrows) 
{
   memcpy( C, A, sizeof(double) * (nrows * ncols) );
   memcpy( C + (nrows*ncols), B, sizeof(double) * (mrows * ncols) );
}
