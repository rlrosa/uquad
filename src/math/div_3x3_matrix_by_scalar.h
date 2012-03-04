////////////////////////////////////////////////////////////////////////////////
// File: div_3x3_matrix_by_scalar.h                                           //
// Routine(s):                                                                //
//    Divide_3x3_Matrix_by_Scalar                                             //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Divide_3x3_Matrix_by_Scalar(double *A, double x)                     //
//                                                                            //
//  Description:                                                              //
//     Divide each element of the matrix A by the scalar x.                   //
//     i.e.       A[i][j] <- A[i][j] / x for all i,j.                         //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     double x     Scalar which divides each element of the matrix A.        //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     double A[3][3],  x;                                                    //
//                                                                            //
//     (your code to initialize the matrix A and scalar x)                    //
//                                                                            //
//     if (x != 0.0) Divide_3x3_Matrix_by_Scalar(&A[0][0], x);                //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////

#define Divide_3x3_Matrix_by_Scalar(A,x) {double z=1.0/x;double*pA=(double*)A;\
pA[0] *= z; pA[1] *= z; pA[2] *= z; pA[3] *= z; pA[4] *= z; pA[5] *= z;\
pA[6] *= z; pA[7] *= z; pA[8] *= z;}
