////////////////////////////////////////////////////////////////////////////////
// File: mul_matrix_by_scalar.c                                               //
// Routine(s):                                                                //
//    Multiply_Matrix_by_Scalar                                               //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Multiply_Matrix_by_Scalar(double *A, double x, int nrows, int ncols) //
//                                                                            //
//  Description:                                                              //
//     Multiply each element of the matrix A by the scalar x.                 //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     double x     Scalar to multipy each element of the matrix A.           //
//     int    nrows The number of rows of matrix A.                           //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N],  x;                                                    //
//                                                                            //
//     (your code to initialize the matrix A and scalar x)                    //
//                                                                            //
//     Multiply_Matrix_by_Scalar(&A[0][0], x, M, N);                          //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Multiply_Matrix_by_Scalar(double *A, double x, int nrows, int ncols) 
{
   register int n = nrows * ncols - 1;
   
   for (; n >= 0; n--) A[n] *= x;
}
