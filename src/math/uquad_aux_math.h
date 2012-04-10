#ifndef UQUAD_AUX_MATH_H
#define UQUAD_AUX_MATH_H

#include <uquad_error_codes.h>
#include <uquad_types.h>

#define UQUAD_MATH_MAX_DIM 256
#define UQUAD_MAT_MAX_DIM 64

#define USE_EQUILIBRATE 0

//TODO poner cuenta
#define deg2rad(a) (a*PI/180.0)
#define rad2deg(a) (a*180.0/PI)
#define uquad_round_double2int(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#define uquad_max(a,b) (a>b)?a:b
#define uquad_min(a,b) (a<b)?a:b
#define uquad_square(a) (a*a)

/// abs() casts to integer, we don't want that
#define uquad_abs(a) (((a) < 0) ? -(a) : (a))

/// Math in degrees (instead of rad)
#define cosd(a) cos(deg2rad(a))
#define sind(a) sin(deg2rad(a))
#define tand(a) atan(deg2rad(a))
#define atan2d(y,x) atan2(deg2rad(y),deg2rad(x))

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

int uquad_mat_prod(uquad_mat_t *C, uquad_mat_t *A,uquad_mat_t *B);

int uquad_mat_det(uquad_mat_t *m, double *res);

int uquad_mat_scalar_mul(uquad_mat_t *Mk, uquad_mat_t *M, double k);
int uquad_mat_scalar_div(uquad_mat_t *Mk, uquad_mat_t *M, double k);

int uquad_mat_sub(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);
int uquad_mat_add(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);

int uquad_solve_lin(uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *x, uquad_mat_t *maux);

int uquad_mat_inv(uquad_mat_t *Minv, uquad_mat_t *M, uquad_mat_t *Meye, uquad_mat_t *Maux);

int uquad_mat_transpose(uquad_mat_t *Mt, uquad_mat_t *M);
int uquad_mat_transpose_inplace(uquad_mat_t *m);

int uquad_mat_dot_product(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);

int uquad_mat_load(uquad_mat_t *m, FILE *input);
void uquad_mat_dump(uquad_mat_t *m, FILE *output);

int uquad_mat_eye(uquad_mat_t *m);
int uquad_mat_zeros(uquad_mat_t *m);

int uquad_mat_get_subm(uquad_mat_t *S, int r, int c, uquad_mat_t *A);
int uquad_mat_set_subm(uquad_mat_t *A, int r, int c, uquad_mat_t *S);
int uquad_mat_copy(uquad_mat_t *dest, uquad_mat_t *src);

int uquad_mat_exp(uquad_mat_t *expA, uquad_mat_t *A);
double uquad_mat_norm( uquad_mat_t *A);
int uquad_mat_int(uquad_mat_t *B, uquad_mat_t *A, double ti, double tf, double step);
int uquad_mat_mirror_rows(uquad_mat_t *H, uquad_mat_t *S, int n);

int uquad_mat_rotate(uquad_mat_t *Vr, uquad_mat_t *V, double phi, double psi, double theta);

uquad_mat_t *uquad_mat_alloc(int r, int c);
void uquad_mat_free(uquad_mat_t *m);

#endif //UQUAD_AUX_MATH_H
