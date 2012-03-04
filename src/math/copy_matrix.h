////////////////////////////////////////////////////////////////////////////////
// File: copy_matrix.h                                                        //
// Routine(s):                                                                //
//    Copy_Matrix                                                             //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Copy_Matrix(double *A, double *B, int nrows, int ncols)              //
//                                                                            //
//  Description:                                                              //
//     Copy the nrows x ncols matrix B to the nrows x ncols matrix A.         //
//     i.e.    A = B.                                                         //
//     The memory locations of the source matrix, B, and the destination      //
//     matrix, A, must not overlap, otherwise the results are installation    //
//     dependent.                                                             //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the destination matrix    //
//                  A[nrows][ncols].                                          //
//     double *B    Pointer to the first element of the source matrix         //
//                  B[nrows][ncols].                                          //
//     int    nrows The number of rows matrices A and B.                      //
//     int    ncols The number of columns of the matrices A and B.            //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[N][M],  B[N][M];                                              //
//                                                                            //
//     (your code to initialize the matrix B)                                 //
//                                                                            //
//     Copy_Matrix(&A[0][0], &B[0][0], N, M);                                 //
//     printf(" Matrix A is \n");                                             //
////////////////////////////////////////////////////////////////////////////////

#include <string.h>                                 // required for memcpy()

#define Copy_Matrix(A, B, m, n) ( memcpy((A), (B), sizeof(double)*(m)*(n)))
