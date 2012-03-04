#ifndef UQUAD_AUX_MATH_H
#define UQUAD_AUX_MATH_H

#define UQUAD_MATH_MAX_DIM 256
#define UQUAD_MAT_MAX_DIM 64

#include <uquad_error_codes.h>
#include <uquad_types.h>

/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Vector
 * -- -- -- -- -- -- -- -- -- -- -- --
 */
struct uquad_vec {
    double * v;      // elements
    int l;           // len
};
typedef struct uquad_vec uquad_vec_t;

int uquad_vec_dot(uquad_vec_t *v1, uquad_vec_t *v2, uquad_vec_t *vr);

int uquad_vec_cross(uquad_vec_t *v1, uquad_vec_t *v2, uquad_vec_t *vr);

uquad_vec_t *uquad_vec_alloc(int l);
void uquad_vec_free(uquad_vec_t *v);

/**
 * -- -- -- -- -- -- -- -- -- -- -- --
 * Matrix
 * -- -- -- -- -- -- -- -- -- -- -- --
 */
struct uquad_mat {
    double ** m;     // elements as [er,ec]: m[row][col]
    double * m_full; // elements as [er,ec]: m[m->c*er + ec]
    int r;           // rows
    int c;           // columns
};
typedef struct uquad_mat uquad_mat_t;

int uquad_mat_prod(uquad_mat_t *m1,uquad_mat_t *m2,uquad_mat_t *mr);

int uquad_mat_det(uquad_mat_t *m, double *res);

int uquad_mat_scalar_div(uquad_mat_t *m, double k);
int uquad_mat_scalar_mul(uquad_mat_t *m, double k);

int uquad_mat_sub(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);
int uquad_mat_add(uquad_mat_t *C, uquad_mat_t *A, uquad_mat_t *B);

int uquad_solve_lin(uquad_mat_t *A, uquad_mat_t *B, uquad_mat_t *x, uquad_mat_t *maux);

int uquad_mat_inv(uquad_mat_t *m1, uquad_mat_t *mr, uquad_mat_t *maux);

void uquad_mat_dump(uquad_mat_t *m, FILE *output);

uquad_mat_t *uquad_mat_alloc(int r, int c);
void uquad_mat_free(uquad_mat_t *m);

#endif //UQUAD_AUX_MATH_H
