////////////////////////////////////////////////////////////////////////////////
// File: set_diagonal.c                                                       //
// Routine(s):                                                                //
//    Set_Diagonal                                                            //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Set_Diagonal(double *A, double v[], int nrows, int ncols)            //
//                                                                            //
//  Description:                                                              //
//     Copy the vector v to the diagonal A[i][i], where 0 <= i <              //
//     min( nrows, ncols ).                                                   //
//     Note that v should be declared "double v[N]", N >= min( nrows, ncols ) //
//     in the calling routine.                                                //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the source matrix A.      //
//     double v[]   Source of the new diagonal for the matrix A.              //
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
//     (your code to initialize the matrix A and the vector v)                //
//                                                                            //
//     Set_Diagonal(&A[0][0], v, M, N);                                       //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Set_Diagonal(double *A, double v[], int nrows, int ncols)
{
   int n;

   if (nrows < ncols) n = nrows; else n = ncols;

   for (; n > 0 ; A += (ncols + 1), n--)  *A = *v++;
}
