/**
 * uquad_aux_math: lib for matrix math
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   uquad_aux_math.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:08:44 2012
 *
 * @brief  lib for matrix math
 *
 * Most of the implementations were taken from code by:
 *    Dick Horn - mathretprogr at gmail com
 *    http://www.mymathlib.com
 * We are truly thankfull to him for his contribution.
 *
 * Examples:
 *   - src/main/main.c
 *   - src/test/math/math_test.c
 *   - src/imu/imu_comm.c
 *   - src/kalman/uquad_kalman.c
 */
#ifndef UQUAD_AUX_MATH_H
#define UQUAD_AUX_MATH_H

#include <uquad_error_codes.h>
#include <uquad_types.h>

#define UQUAD_MATH_MAX_DIM 256
#define UQUAD_MAT_MAX_DIM 64

#define USE_EQUILIBRATE 0

//TODO poner cuenta
#define deg2rad(a)                (a*PI/180.0)
#define rad2deg(a)                (a*180.0/PI)
#define uquad_round_double2int(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define fix(a)                    ((a>0)?floor(a):ceil(a))
#define sign(a)                   ((a < 0.0)?-1.0:1.0)
#define uquad_max(a,b)            ((a>b)?a:b)
#define uquad_min(a,b)            ((a<b)?a:b)
#define uquad_square(a)           (a*a)

/// abs() casts to integer, we don't want that
#define uquad_abs(a)              (((a) < 0) ? -(a) : (a))

/// Math in degrees (instead of rad)
#define cosd(a)                   cos(deg2rad(a))
#define sind(a)                   sin(deg2rad(a))
#define tand(a)                   atan(deg2rad(a))
#define atan2d(y,x)               atan2(deg2rad(y),deg2rad(x))

/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Dinamically allocated matrix
 *
 * Documentation in uquad_aux_math.c
 *
 * Example of usage:
 *   int i,j,retval;
 *   uquad_mat_t *A = uquad_mat_alloc(3,1);
 *   uquad_mat_t *B = uquad_mat_alloc(1,3);
 *   uquad_mat_t *C = uquad_mat_alloc(3,3);
 *   if (A == NULL || B == NULL || C == NULL)
 *   {
 *     perror("Failed to allocate mem");
 *     exit(-1);
 *   }
 *   for(i=0; i < A->r*A->c; ++i)
 *     A->m_full[i] = rand();
 *   for(i=0; i < B->r; ++i)
 *     for(j=0; j < B->c; ++j)
 *       B->m[i][j];
 *   retval = uquad_mat_prod(C,A,B);
 *   if(retval != ERROR_OK)
 *   {
 *     err_propagate(retval);
 *   }
 *   uquad_mat_dump(C, stdout);
 *   uquad_mat_free(A);
 *   uquad_mat_free(B);
 *   uquad_mat_free(C);
 *   exit(0);
 * -- -- -- -- -- -- -- -- -- -- -- --
 */
struct uquad_mat {
    double ** m;     // elements as [er,ec]: m[row][col]
    double * m_full; // elements as [er,ec]: m[m->c*er + ec]
    int r;           // rows
    int c;           // columns
};
typedef struct uquad_mat uquad_mat_t;

/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Matrix
 * -- -- -- -- -- -- -- -- -- -- -- --
 */

/**
 * Performs:
 *   C = A*B
 *
 * Checks dimensions before performing operation.
 * Memory for C,A and B must have been previously allocated.
 *
 * If matrices are 3x3, a macro is used (+ efficiency).
 *
 * @param C Result.
 * @param A First operand.
 * @param B Second operand.
 *
 * @return Error code.
 */
int uquad_mat_prod(uquad_mat_t *C, uquad_mat_t *A,uquad_mat_t *B);

int uquad_mat_det(uquad_mat_t *m, double *res);

/**
 * Multiplies(divides) matrix m by scalar value k.
 * If only one matrix is supplied, will multiply(divide) in place.
 *
 * If in place and matrix is 3x3, a macro is used (+efficiency)
 *
 * @param Mk Result will be returned here.
 * @param M  NULL for in place multiplicacion(division). Otherwise, M will remain
 * unmodified, and the anwser will be Mk = M*k(M/k)
 * @param k
 *
 * @return error code
 */
int uquad_mat_scalar_mul(uquad_mat_t *Mk, uquad_mat_t *M, double k);
int uquad_mat_scalar_div(uquad_mat_t *Mk, uquad_mat_t *M, double k);

/**
 * Performs:
 *   C = A-B (A+B)
 *
 * Checks dimensions before performing operation.
 * Memory for C,A and B must have been previously allocated.
 *
 * If matrices are 3x3, a macro is used (+ efficiency).
 *
 * @param C Result.
 * @param A First operand.
 * @param B Second operand.
 *
 * @return Error code.
 */
int uquad_mat_sub(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);
int uquad_mat_add(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);

/**
 * Solves linear system using gaussian elimination:
 *   A*x = B
 *
 * Checks dimensions before operating.
 * Requires aux matrix to build [A:B]. If none is supplied, will
 * allocate required memory, and then free it.
 *
 * @param A
 * @param B
 * @param x
 * @param maux NULL or aux memory for [A:B], content destroyed.
 *
 * @return Error code.
 */
int uquad_solve_lin(uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *x, uquad_mat_t *maux);

/**
 * Solves a system a*x^2 + b*x + c for real values.
 *
 * @param xp If not NULL, then xp = -b + ...
 * @param xm If not NULL, then xp = -b - ...
 * @param a
 * @param b
 * @param c
 *
 * @return error code.
 */
int uquad_solve_pol2(double *xp, double *xm, double a, double b, double c);

/**
 * Inverts matrix.
 * Assumes memory was previously allocated for minv.
 * Requires aux memory, two matrices. If not supplied, will allocate and
 * free after finishing.
 *
 * @param Minv Result.
 * @param M Input.
 * @param Meye NULL or auxiliary matrix, size of M.
 * @param Maux NULL or auxiliary matrix, size of [M:M]
 *
 * @return Error code.
 */
int uquad_mat_inv(uquad_mat_t *Minv, uquad_mat_t *M, uquad_mat_t *Meye, uquad_mat_t *Maux);

/**
 * Transposes a matrix
 *
 * @param Mt transpose of M
 * @param M input
 *
 * @return error code
 */
int uquad_mat_transpose(uquad_mat_t *Mt, uquad_mat_t *M);

/**
 * Transposes a matrix, in place.
 *
 * @param m is the input, and will be transposed after execution.
 *
 * @return error code
 */
int uquad_mat_transpose_inplace(uquad_mat_t *m);

/**
 * Performs element to element product of matrices:
 *  C[i][j] = A[i][j]*B[i][j] for all i,j
 * If called with A==NULL and B==NULL, will square each
 * element of C, in place (C will be modified after execution)
 *
 * @param C answer
 * @param A input or NULL
 * @param B input or NULL
 *
 * @return error code
 */
int uquad_mat_dot_product(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);

/**
 * Will load matrix from text file.
 * Will ignore spaces, end of lines, tabs, etc.
 * Assumes memory has been previously allocated for matrix.
 *
 * Example of a configuration file for a 7x3 matrix:
 *   26.9552751514508        0                       0
 *   0                       27.1044248763245        0
 *   0                       0                       25.9160276670631
 *
 *   1                       0.00953542215888624       0.00955723285532017
 *   -0.000376784971686033   1                         -0.00677033632701969
 *   0.0105047129596497      -0.00456172876850397      1
 *
 *   13.0716873433579
 *
 *   -1.07951072069862
 *   -40.723583688652
 *
 * Note that elements are read row-wise (fills row, then next row, etc)
 *
 * @param imu
 * @param input stream to load from, or NULL for stdin
 *
 * @return
 */
int uquad_mat_load(uquad_mat_t *m, FILE *input);

/**
 * Prints matrix to any output.
 *
 * @param m
 * @param output
 */
void uquad_mat_dump(uquad_mat_t *m, FILE *output);

/**
 * Prints vector matrix to any output,
 * in a single line.
 *
 * @param m
 * @param output
 * @param tight Compress output at the cost of loosing info.
 */
void uquad_mat_dump_vec(uquad_mat_t *m, FILE *output, uquad_bool_t tight);

/**
 * Sets matrix to identity.
 * Assumes memory was previously allocated.
 *
 * @param m
 *
 * @return Error code
 */
int uquad_mat_eye(uquad_mat_t *m);

/**
 * Fills a matrix with zeros.
 *
 * @param m
 *
 * @return error code.
 */
int uquad_mat_zeros(uquad_mat_t *m);

/**
 * Fills a matrix with val
 *
 * @param m
 * @param val
 *
 * @return error code.
 */
int uquad_mat_fill(uquad_mat_t *m, double val);

/**
 * Creates a diagonal matrix, setting diagonal of m to diag, and the
 * rest to zeros.
 * Assumes length of diag matches length of diagonal.
 *
 * @param m
 * @param diag
 *
 * @return
 */
int uquad_mat_diag(uquad_mat_t *m, double *diag);
int uquad_mat_get_diag(double v[],uquad_mat_t *m,int n);

/**
 * Copies part of a matrix A to a smaller matrix, S.
 *
 * @param S answer is returned here
 * @param r first row of A that will be copied to S
 * @param c first column of A that will be copied to S
 * @param A src matrix
 *
 * @return
 */
int uquad_mat_get_subm(uquad_mat_t *S, int r, int c, uquad_mat_t *A);

/**
 * Sets part of a matrix A to match a smaller matrix, S.
 *
 * @param A answer is returned here.
 * @param r first row of A where S will be copied to.
 * @param c first column of A where S will be copied to.
 * @param S submatrix
 *
 * @return
 */
int uquad_mat_set_subm(uquad_mat_t *A, int r, int c, uquad_mat_t *S);

/**
 * Copy matrix src to dest.
 * Assumes memory has been previously allocated for dest.
 *
 * @param dest
 * @param src
 *
 * @return error code.
 */
int uquad_mat_copy(uquad_mat_t *dest, uquad_mat_t *src);

/**
 * Computes matrix exponencial:
 *   expA = e^(A)
 * 
 * @param expA 
 * @param A 
 * 
 * @return 
 */
int uquad_mat_exp(uquad_mat_t *expA, uquad_mat_t *A);

/**
 * Performs the euclidean norm of a matrix:
 *  sqrt(sum(a[0]*a[0]+a[1]*a[1]...)
 *
 * @param A
 *
 * @return Answer or -1.0 if error.
 */
double uquad_mat_norm( uquad_mat_t *A);

/**
 * Performs the integral of A^t for ti<t<tf with an integration step of step.
 *
 * @param B Ans.
 * @param A
 * @param ti
 * @param tf
 * @param step
 *
 * @return
 */
int uquad_mat_int(uquad_mat_t *B, uquad_mat_t *A, double ti, double tf, double step);

/**
 * Builds a rotation matrix R from phi,psi,theta, and performs Vr = R*V
 *
 *   R_psi:
 *   +-                        -+
 *   |  1,     0,         0     |
 *   |                          |
 *   |  0,  cos(psi), sin(psi)  |
 *   |                          |
 *   |  0, -sin(psi), cos(psi)  |
 *   +-                        -+
 *
 *   R_phi:
 *   +-                        -+
 *   |  cos(phi), 0, -sin(phi)  |
 *   |                          |
 *   |      0,    1,     0      |
 *   |                          |
 *   |  sin(phi), 0,  cos(phi)  |
 *   +-                        -+
 *
 *   R_theta:
 *   +-                            -+
 *   |   cos(theta), sin(theta), 0  |
 *   |                              |
 *   |  -sin(theta), cos(theta), 0  |
 *   |                              |
 *   |       0,           0,     1  |
 *   +-                            -+
 *
 *   R = R_psi*R_phi*R_theta:
 *   +-                                                                                                                           -+
 *   |                  cos(phi) cos(theta),                                cos(phi) sin(theta),                    -sin(phi)      |
 *   |                                                                                                                             |
 *   |  cos(theta) sin(phi) sin(psi) - cos(psi) sin(theta), cos(psi) cos(theta) + sin(phi) sin(psi) sin(theta), cos(phi) sin(psi)  |
 *   |                                                                                                                             |
 *   |  sin(psi) sin(theta) + cos(psi) cos(theta) sin(phi), cos(psi) sin(phi) sin(theta) - cos(theta) sin(psi), cos(phi) cos(psi)  |
 *   +-                                                                                                                           -+
 *
 *   if from_inertial, use R = R_theta_2*R_phi_2*R_psi_2 to rotate:
 *   +-                                                                                                                             -+
 *   |  cos(phi)*cos(theta), cos(theta)*sin(phi)*sin(psi) - cos(psi)*sin(theta), sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi)  |
 *   |                                                                                                                               |
 *   |  cos(phi)*sin(theta), cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi)*sin(theta) - cos(theta)*sin(psi)  |
 *   |                                                                                                                               |
 *   |            -sin(phi),                                  cos(phi)*sin(psi),                                  cos(phi)*cos(psi)  |
 *   +-                                                                                                                             -+\
 *   Note: R_theta_2, R_phi_2, R_psi_2 rotate using -theta, -phi, -psi
 *
 * @param from_inertial
 * @param Vr : answer
 * @param V
 * @param phi
 * @param psi
 * @param theta
 * @param R NULL or aux mem for rotation, avoids malloc/free
 *
 * @return
 */
int uquad_mat_rotate(uquad_bool_t from_inertial,
		     uquad_mat_t *Vr, uquad_mat_t *V,
		     double psi, double phi, double theta,
		     uquad_mat_t *R);

/**
 * Allocates matrix of size r rows and c columns.
 *
 * @param r Rows.
 * @param c Columns.
 *
 * @return NULL or pointer to allocated memory.
 */
uquad_mat_t *uquad_mat_alloc(int r, int c);

/**
 * Free memory allocated my uquad_mat_alloc()
 * Will check if NULL argument is supplied.
 *
 * @param m
 */
void uquad_mat_free(uquad_mat_t *m);

#endif //UQUAD_AUX_MATH_H
