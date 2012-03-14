#include "control.h"
#include <stdlib.h>
#include <uquad_error_codes.h>

uquad_mat_t *tmp_sub_sp_x;
uquad_mat_t *tmp_x_hat_partial;
ctrl_t *control_init(void)
{
    ctrl_t *ctrl = (ctrl_t *)malloc(sizeof(ctrl_t));
    mem_alloc_check(ctrl);
    ctrl->K = uquad_mat_alloc(4,7);
    tmp_sub_sp_x = uquad_mat_alloc(12,1);
    tmp_x_hat_partial = uquad_mat_alloc(7,1);
    if(ctrl->K == NULL || tmp_sub_sp_x == NULL || tmp_x_hat_partial == NULL)
    {
	err_log("Failed to allocate gain matrix!");
	control_deinit(ctrl);
	return NULL;
    }
    ctrl->K->m_full[0] = 5;
    ctrl->K->m_full[1] = -1.55703484873226e-16;
    ctrl->K->m_full[2] = -0.707106781186547;
    ctrl->K->m_full[3] = 12.9225415806597;
    ctrl->K->m_full[4] = -2.1254792876218e-16;
    ctrl->K->m_full[5] = -1.71708266311789;
    ctrl->K->m_full[6] = 0.5;

    ctrl->K->m_full[7] = 5;
    ctrl->K->m_full[8] = 0.707106781186548;
    ctrl->K->m_full[9] = 1.08226885949043e-16;
    ctrl->K->m_full[10] = 12.9225415806597;
    ctrl->K->m_full[11] = 1.7170826631179;
    ctrl->K->m_full[12] = 2.132882808075e-16;
    ctrl->K->m_full[13] = -0.5;

    ctrl->K->m_full[14] = 5;
    ctrl->K->m_full[15] = 1.55703484873226e-16;
    ctrl->K->m_full[16] = 0.707106781186547;
    ctrl->K->m_full[17] = 12.9225415806597;
    ctrl->K->m_full[18] = 2.1254792876218e-16;
    ctrl->K->m_full[19] = 1.71708266311789;
    ctrl->K->m_full[20] = 0.5;

    ctrl->K->m_full[21] = 5;
    ctrl->K->m_full[22] = -0.707106781186548;
    ctrl->K->m_full[23] = -9.76657201075811e-17;
    ctrl->K->m_full[24] = 12.9225415806597;
    ctrl->K->m_full[25] = -1.7170826631179;
    ctrl->K->m_full[26] = -2.1180757671686e-16;
    ctrl->K->m_full[27] = -0.5;

    return ctrl;
}

#if DEBUG
FILE *h_hat_partial_log = NULL;
uquad_mat_t *h_hat_t = NULL;
#endif
int control(ctrl_t *ctrl, uquad_mat_t *w, uquad_mat_t *x, set_point_t *sp)
{
    int retval;
    if(ctrl == NULL || w == NULL || x == NULL || sp == NULL)
    {
	err_check(ERROR_NULL_POINTER,"Inputs must be non NULL!");
    }

    retval = uquad_mat_sub(tmp_sub_sp_x, sp->x, x);
    err_propagate(retval);

    /// only using part of the state vector
    tmp_x_hat_partial->m_full[0] = tmp_sub_sp_x->m_full[2];  // z
    tmp_x_hat_partial->m_full[1] = tmp_sub_sp_x->m_full[3];  // psi
    tmp_x_hat_partial->m_full[2] = tmp_sub_sp_x->m_full[4];  // phi
    tmp_x_hat_partial->m_full[3] = tmp_sub_sp_x->m_full[8];  // vqz
    tmp_x_hat_partial->m_full[4] = tmp_sub_sp_x->m_full[9];  // wqx
    tmp_x_hat_partial->m_full[5] = tmp_sub_sp_x->m_full[10]; // wqy
    tmp_x_hat_partial->m_full[6] = tmp_sub_sp_x->m_full[11]; // wqz

#if DEBUG
    if(h_hat_partial_log == NULL)
    {
	h_hat_partial_log = fopen("h_partial.log","w");
	h_hat_t = uquad_mat_alloc(1,7);
    }
    retval = uquad_mat_transpose(h_hat_t, tmp_x_hat_partial);
    uquad_mat_dump(h_hat_t,h_hat_partial_log);
#endif

    retval = uquad_mat_prod(w, ctrl->K, tmp_x_hat_partial);
    err_propagate(retval);
    retval = uquad_mat_add(w,sp->w,w);
    err_propagate(retval);

    return ERROR_OK;
}

void control_deinit(ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
	err_log("No memory had been allocated for ctrl");
	return;
    }
    uquad_mat_free(ctrl->K);
    uquad_mat_free(tmp_sub_sp_x);
    uquad_mat_free(tmp_x_hat_partial);
    free(ctrl);
}

