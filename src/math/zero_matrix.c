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
