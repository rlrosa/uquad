////////////////////////////////////////////////////////////////////////////////
// File: gauss_aux_elimination.c                                              //
// Routines:                                                                  //
//    Gaussian_Elimination_Aux                                                //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  int Gaussian_Elimination_Aux(double *AB, int nrows, int ncols)            //
//                                                                            //
//     Solve the linear system of equations Ax=B where A is an n x n matrix   //
//     B is an n x m matrix for the n x m matrix x.  The argument AB is the   //
//     auxillary matrix [A,B], an n x (n+m) matrix in which the first n       //
//     columns are the n columns of the matrix A followed by the m columns of //
//     the matrix B.                                                          //
//                                                                            //
//     This routine performs partial pivoting and the elements of AB are      //
//     modified during computation.                                           //
//     If the matrix A is singular, the return value of the function call is  //
//     either -1. If the solution was found, the function return value is 0.  //
//                                                                            //
//  Arguments:                                                                //
//     double *AB     On input, the pointer to the first element of the       //
//                    auxillary matrix AB[nrows][ncols].  On output, the      //
//                    auxillary matrix AB[nrows][ncols] is destroyed, and the //
//                    solution is in the last ncols - nrows columns of AB.    //
//     int     nrows  The number of rows of the auxillary matrix AB.          //
//     int     ncols  The number of columns of the auxillary matrix AB.       //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define NROWS                                                          //
//     #define NCOLS                                                          //
//     double AB[NROWS][NCOLS];                                               //
//                                                                            //
//     (your code to create the auxillary matrix AB)                          //
//     err = Gaussian_Elimination_Aux((double*)AB, NROWS, NCOLS);             //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else { printf(" The Solution is: \n"); ...                             //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
#include <math.h>                                     // required for fabs()

int Gaussian_Elimination_Aux(double *A, int nrows, int ncols)
{
   int row, i, j;
   double max, dum, *pa, *pA, *pivot_row, *px;

                     // for each row find scale factor

   for (row = 0, pa = A; row < (nrows - 1); row++, pa += ncols) {

//            find the pivot row

      pivot_row = pa;
      max = fabs(*(pa + row));
      for (pA = pa + ncols, j = row + 1; j < nrows; pA += ncols, j++)
         if ((dum = fabs(*(pA + row))) > max) { max = dum; pivot_row = pA; }
      if (max == 0.0) return -1;

//            and if it differs from the current row, interchange the two rows.
             
      if (pivot_row != pa) {
         for (j = row; j < ncols; j++) {
            dum = *(pa + j);
            *(pa + j) = *(pivot_row + j);
            *(pivot_row + j) = dum;
         }
      }

//             Perform forward substitution

      for (i = row + 1; i < nrows; i++) {
         pA = A + i * ncols;
         dum = - *(pA + row) / *(pa + row);
         *(pA + row) = 0.0;
         for (j = row + 1; j < ncols; j++) *(pA + j) += dum * *(pa + j);
      }
   }

//              Perform backward substitution
  
   pa = A + (nrows - 1) * ncols;
   for (row = nrows - 1; row >= 0; pa -= ncols, row--) {
      if ( *(pa + row) == 0.0 ) return -1;           // matrix is singular
      dum = 1.0 / *(pa + row);
      for ( i = row + 1; i < ncols; i++) *(pa + i) *= dum; 
      for ( i = 0, pA = A; i < row; pA += ncols, i++) {
         dum = *(pA + row);
         for ( j = row + 1; j < ncols; j++) *(pA + j) -= dum * *(pa + j);
      }
   }
   return 0;
}
