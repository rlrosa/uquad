////////////////////////////////////////////////////////////////////////////////
// File: doolittle.c                                                          //
// Routines:                                                                  //
//    Doolittle_LU_Decomposition                                              //
//    Doolittle_LU_Solve                                                      //
//                                                                            //
// Required Externally Defined Routines:                                      //
//    Unit_Lower_Triangular_Solve                                             //
//    Upper_Triangular_Solve                                                  //
////////////////////////////////////////////////////////////////////////////////

//                    Required Externally Defined Routines 
void Unit_Lower_Triangular_Solve(double *L, double B[], double x[], int n);
int  Upper_Triangular_Solve(double *U, double B[], double x[], int n);

////////////////////////////////////////////////////////////////////////////////
//  int Doolittle_LU_Decomposition(double *A, int n)                          //
//                                                                            //
//  Description:                                                              //
//     This routine uses Doolittle's method to decompose the n x n matrix A   //
//     into a unit lower triangular matrix L and an upper triangular matrix U //
//     such that A = LU.                                                      //
//     The matrices L and U replace the matrix A so that the original matrix  //
//     A is destroyed.                                                        //
//     Note!  In Doolittle's method the diagonal elements of L are 1 and are  //
//            not stored.                                                     //
//     Note!  The determinant of A is the product of the diagonal elements    //
//            of U.  (det A = det L * det U = det U).                         //
//     This routine is suitable for those classes of matrices which when      //
//     performing Gaussian elimination do not need to undergo partial         //
//     pivoting, e.g. positive definite symmetric matrices, diagonally        //
//     dominant band matrices, etc.                                           //
//     For the more general case in which partial pivoting is needed use      //
//                  Doolittle_LU_Decomposition_with_Pivoting.                 //
//     The LU decomposition is convenient when one needs to solve the linear  //
//     equation Ax = B for the vector x while the matrix A is fixed and the   //
//     vector B is varied.  The routine for solving the linear system Ax = B  //
//     after performing the LU decomposition for A is Doolittle_LU_Solve      //
//     (see below).                                                           //
//                                                                            //
//     The Doolittle method is given by evaluating, in order, the following   //
//     pair of expressions for k = 0, ... , n-1:                              //
//       U[k][j] = A[k][j] - (L[k][0]*U[0][j] + ... + L[k][k-1]*U[k-1][j])    //
//                                 for j = k, k+1, ... , n-1                  //
//       L[i][k] = (A[i][k] - (L[i][0]*U[0][k] + . + L[i][k-1]*U[k-1][k]))    //
//                          / U[k][k]                                         //
//                                 for i = k+1, ... , n-1.                    //
//       The matrix U forms the upper triangular matrix, and the matrix L     //
//       forms the lower triangular matrix.                                   //
//                                                                            //
//  Arguments:                                                                //
//     double *A   Pointer to the first element of the matrix A[n][n].        //
//     int     n   The number of rows or columns of the matrix A.             //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N];                                                        //
//                                                                            //
//     (your code to intialize the matrix A)                                  //
//                                                                            //
//     err = Doolittle_LU_Decomposition(&A[0][0], N);                         //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else { printf(" The LU decomposition of A is \n");                     //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Doolittle_LU_Decomposition(double *A, int n)
{
   int i, j, k, p;
   double *p_k, *p_row, *p_col;

//         For each row and column, k = 0, ..., n-1,
//            find the upper triangular matrix elements for row k
//            and if the matrix is non-singular (nonzero diagonal element).
//            find the lower triangular matrix elements for column k. 
 
   for (k = 0, p_k = A; k < n; p_k += n, k++) {
      for (j = k; j < n; j++) {
         for (p = 0, p_col = A; p < k; p_col += n,  p++)
            *(p_k + j) -= *(p_k + p) * *(p_col + j);
      }
      if ( *(p_k + k) == 0.0 ) return -1;
      for (i = k+1, p_row = p_k + n; i < n; p_row += n, i++) {
         for (p = 0, p_col = A; p < k; p_col += n, p++)
            *(p_row + k) -= *(p_row + p) * *(p_col + k);
         *(p_row + k) /= *(p_k + k);
      }  
   }
   return 0;
}


////////////////////////////////////////////////////////////////////////////////
//  int Doolittle_LU_Solve(double *LU, double *B, double *x,  int n)           //
//                                                                            //
//  Description:                                                              //
//     This routine uses Doolittle's method to solve the linear equation      //
//     Ax = B.  This routine is called after the matrix A has been decomposed //
//     into a product of a unit lower triangular matrix L and an upper        //
//     triangular matrix U without pivoting.  The argument LU is a pointer to //
//     the matrix the subdiagonal part of which is L and the superdiagonal    //
//     together with the diagonal part is U. (The diagonal part of L is 1 and //
//     is not stored.)   The matrix A = LU.                                   //
//     The solution proceeds by solving the linear equation Ly = B for y and  //
//     subsequently solving the linear equation Ux = y for x.                 //
//                                                                            //
//  Arguments:                                                                //
//     double *LU  Pointer to the first element of the matrix whose elements  //
//                 form the lower and upper triangular matrix factors of A.   //
//     double *B   Pointer to the column vector, (n x 1) matrix, B            //
//     double *x   Solution to the equation Ax = B.                           //
//     int     n   The number of rows or columns of the matrix LU.            //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Doolittle_LU_Decomposition(&A[0][0], N);                         //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else {                                                                 //
//        err = Doolittle_LU_Solve(&A[0][0], B, x, n);                        //
//        if (err < 0) printf(" Matrix A is singular\n");                     //
//        else printf(" The solution is \n");                                 //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Doolittle_LU_Solve(double *LU, double B[], double x[], int n)
{

//         Solve the linear equation Lx = B for x, where L is a lower
//         triangular matrix with an implied 1 along the diagonal.
   
   Unit_Lower_Triangular_Solve(LU, B, x, n);

//         Solve the linear equation Ux = y, where y is the solution
//         obtained above of Lx = B and U is an upper triangular matrix.

   return Upper_Triangular_Solve(LU, x, x, n);
}
