#ifndef UQUAD_AUX_MATH_H
#define UQUAD_AUX_MATH_H

#define UQUAD_MAT_MAX_DIM 256

#include <uquad_error_codes.h>
#include <uquad_types.h>

struct uquad_mat {
    float ** m; // elements
    int r;          // rows
    int c;          // columns
};
typedef struct uquad_mat uquad_mat_t;

int uquad_mat_prod(uquad_mat_t *m1,uquad_mat_t *m2,uquad_mat_t *mr);

uquad_mat_t *uquad_mat_alloc(int r, int c);

int uquad_mat_free(uquad_mat_t *m);

#endif //UQUAD_AUX_MATH_H
