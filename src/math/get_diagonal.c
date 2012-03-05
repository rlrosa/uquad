////////////////////////////////////////////////////////////////////////////////
// File: get_diagonal.c                                                       //
// Routine(s):                                                                //
//    Get_Diagonal                                                            //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Get_Diagonal(double v[], double *A, int nrows, int ncols)            //
//                                                                            //
//  Description:                                                              //
//     Copy the diagonal A[i][i], i = 0, ..., min( nrows, ncols ) to the      //
//     vector v.                                                              //
//     Note that v should be declared "double v[N]", with                     //
//     N >= min(nrows, ncols) in the calling routine.                         //
//                                                                            //
//  Arguments:                                                                //
//     double v[]   Destination address of the diagonal of the matrix A.      //
//     double *A    Pointer to the first element of the source matrix A.      //
//     int    nrows The number of rows matrix A.                              //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N],  v[N];                                                 //
//                                                                            //
//     (your code to set the matrix A )                                       //
//                                                                            //
//     Get_Diagonal(v, &A[0][0],  M, N);                                      //
//     printf("The diagonal is \n"); ... }                                    //
////////////////////////////////////////////////////////////////////////////////
void Get_Diagonal(double v[], double *A, int nrows, int ncols)
{
   int i, n;

   n = (nrows < ncols) ? nrows: ncols;
   for (i = 0; i < n; A += (ncols + 1), i++) v[i] = *A;
}
