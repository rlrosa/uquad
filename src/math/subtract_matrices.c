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
