////////////////////////////////////////////////////////////////////////////////
// File: transpose_matrix.c                                                   //
// Routine(s):                                                                //
//    Transpose_Matrix                                                        //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Transpose_Matrix(double *At, double *A, int nrows, int ncols)        //
//                                                                            //
//  Description:                                                              //
//     Take the transpose of A and store in At, i.e. At = A'.                 //
//     The matrix At should be declared as double At[ncols][nrows] in the     //
//     calling routine, and the matrix A declared as double A[nrows[ncols].   //
//     In general, At and A should be disjoint i.e. their memory locations    //
//     should be distinct.                                                    //
//                                                                            //
//  Arguments:                                                                //
//     double *At   Pointer to the first element of the matrix At.            //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of matrix A and number of columns of   //
//                  the matrix At.                                            //
//     int    ncols The number of columns of the matrix A and the number of   //
//                  rows of the matrix At.                                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N],  At[N][M];                                             //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//                                                                            //
//     Transpose_Matrix(&At[0][0], &A[0][0], M, N);                           //
//     printf("The transpose of A is the matrix At \n"); ...                  //
////////////////////////////////////////////////////////////////////////////////
void Transpose_Matrix(double *At, double *A, int nrows, int ncols) 
{
   double *pA;
   double *pAt;
   int i,j;

   for (i = 0; i < nrows; At += 1, A += ncols, i++) {
      pAt = At;
      pA = A;
      for (j = 0; j < ncols; pAt += nrows, j++) *pAt = pA[j];
   }
}
