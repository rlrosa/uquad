////////////////////////////////////////////////////////////////////////////////
// File: add_matrices.c                                                       //
// Routine(s):                                                                //
//    Add_Matrices                                                            //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Add_Matrices(double *C, double *A, double *B, int nrows, int ncols)  //
//                                                                            //
//  Description:                                                              //
//     Add matrices A and B to form the matrix C, i.e. C = A + B.             //
//     All matrices should be declared as " double X[nrows][ncols] " in the   //
//     calling routine, where X = A, B, C.                                    //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Address of the first element of the matrix C.             //
//     double *A    Address of the first element of the matrix A.             //
//     double *B    Address of the first element of the matrix B.             //
//     int    nrows The number of rows of matrices A, B, and C.               //
//     int    ncols The number of columns of the matrices A, B, and C.        //
//                                                                            //
//  Return Values:                                                            //
//      void                                                                  //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N],  B[M][N], C[M][N];                                     //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Add_Matrices((double *) C, &A[0][0], &B[0][0], M, N);                  //
//     printf("The matrix C = A + B is \n"); ...                              //
////////////////////////////////////////////////////////////////////////////////
void Add_Matrices(double *C, double *A, double *B, int nrows, int ncols) 
{
   register int i;
   register int n = nrows * ncols;

   for (i = 0; i < n; i++) C[i] = A[i] + B[i];
}
