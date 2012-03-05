////////////////////////////////////////////////////////////////////////////////
// File: mul_3x3_matrix_by_scalar.h                                           //
// Routine(s):                                                                //
//    Multiply_3x3_Matrix_by_Scalar                                           //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Multiply_3x3_Matrix_by_Scalar(double *A, double x)                   //
//                                                                            //
//  Description:                                                              //
//     Multiply each element of the matrix A by the scalar x.                 //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     double x     Scalar to multipy each element of the matrix A.           //
//                                                                            //
//  Return Values:                                                            //
//     Pointer to A.                                                          //
//                                                                            //
//  Example:                                                                  //
//     double A[3][3],  x;                                                    //
//                                                                            //
//     (your code to initialize the matrix A and scalar x)                    //
//                                                                            //
//     Multiply_3x3_Matrix_by_Scalar(&A[0][0], x);                            //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////

#define Multiply_3x3_Matrix_by_Scalar(A,x) {double*pA=(double*)A;\
pA[0] *= x; pA[1] *= x; pA[2] *= x; pA[3] *= x; pA[4] *= x; pA[5] *= x;\
pA[6] *= x; pA[7] *= x; pA[8] *= x;}
