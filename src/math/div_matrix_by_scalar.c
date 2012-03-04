////////////////////////////////////////////////////////////////////////////////
// File: div_matrix_by_scalar.c                                               //
// Routine(s):                                                                //
//    Divide_Matrix_by_Scalar                                                 //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Divide_Matrix_by_Scalar(double *A, double x, int nrows, int ncols)   //
//                                                                            //
//  Description:                                                              //
//     Divide each element of the matrix A by the scalar x.                   //
//     i.e.       A[i][j] <- A[i][j] / x for all i,j.                         //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     double x     Scalar which divides each element of the matrix A.        //
//     int    nrows The number of rows of the matrix A.                       //
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
//     if ( x != 0.0)  Divide_Matrix_by_Scalar(&A[0][0], x, M, N);            //
//      printf("The matrix A is \n"); ...                                     //
////////////////////////////////////////////////////////////////////////////////
void Divide_Matrix_by_Scalar(double *A, double x, int nrows, int ncols) 
{
   int i;
   int n = ncols * nrows;
   double z = 1.0 / x;

   for (i = 0; i < n; i++) A[i] *= z;
}
