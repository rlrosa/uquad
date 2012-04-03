#include "control.h"
#include <stdlib.h>
#include <uquad_error_codes.h>

uquad_mat_t *tmp_sub_sp_x;
uquad_mat_t *tmp_x_hat_partial;
ctrl_t *control_init(void)
{
    ctrl_t *ctrl = (ctrl_t *)malloc(sizeof(ctrl_t));
    mem_alloc_check(ctrl);
    ctrl->K = uquad_mat_alloc(4,STATES_CONTROLLED);
    tmp_sub_sp_x = uquad_mat_alloc(12,1);
    tmp_x_hat_partial = uquad_mat_alloc(STATES_CONTROLLED,1);
    if(ctrl->K == NULL || tmp_sub_sp_x == NULL || tmp_x_hat_partial == NULL)
    {
	err_log("Failed to allocate gain matrix!");
	control_deinit(ctrl);
	return NULL;
    }

    ctrl -> K -> m[0][0] = 0.04997929317406360483;
    ctrl -> K -> m[0][1] = -0.00000000000013907855;
    ctrl -> K -> m[0][2] = -2.21693691096538936236;
    ctrl -> K -> m[0][3] = 1.57522727908218507231;
    ctrl -> K -> m[0][4] = 1.20889778007591086606;
    ctrl -> K -> m[0][5] = -0.00000000000028922617;
    ctrl -> K -> m[0][6] = -2.59308747714005338736;
    ctrl -> K -> m[0][7] = 4.21378644022469739383;

    ctrl -> K -> m[1][0] = 0.04997929317406095417;
    ctrl -> K -> m[1][1] = 2.21693691096555678399;
    ctrl -> K -> m[1][2] = 0.00000000000041898163;
    ctrl -> K -> m[1][3] = -1.57522727908013693288;
    ctrl -> K -> m[1][4] = 1.20889778007587356257;
    ctrl -> K -> m[1][5] = 2.59308747713990550565;
    ctrl -> K -> m[1][6] = 0.00000000000106333779;
    ctrl -> K -> m[1][7] = -4.21378644021892512228;

    ctrl -> K -> m[2][0] = 0.04997929317406046845;
    ctrl -> K -> m[2][1] = 0.00000000000009130094;
    ctrl -> K -> m[2][2] = 2.21693691096482980996;
    ctrl -> K -> m[2][3] = 1.57522727907799686697;
    ctrl -> K -> m[2][4] = 1.20889778007586246034;
    ctrl -> K -> m[2][5] = 0.00000000000024687555;
    ctrl -> K -> m[2][6] = 2.59308747713845866301;
    ctrl -> K -> m[2][7] = 4.21378644021285175825;

    ctrl -> K -> m[3][0] = 0.04997929317406293176;
    ctrl -> K -> m[3][1] = -2.21693691096551415143;
    ctrl -> K -> m[3][2] = 0.00000000000013833003;
    ctrl -> K -> m[3][3] = -1.57522727908005011344;
    ctrl -> K -> m[3][4] = 1.20889778007591908171;
    ctrl -> K -> m[3][5] = -2.59308747713986731398;
    ctrl -> K -> m[3][6] = 0.00000000000052698359;
    ctrl -> K -> m[3][7] = -4.21378644021863024705;

    return ctrl;
}

int control(ctrl_t *ctrl, uquad_mat_t *w, uquad_mat_t *x, set_point_t *sp)
{
    int retval = ERROR_OK;
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
    tmp_x_hat_partial->m_full[3] = tmp_sub_sp_x->m_full[5];  // theta
    tmp_x_hat_partial->m_full[4] = tmp_sub_sp_x->m_full[8];  // vqz
    tmp_x_hat_partial->m_full[5] = tmp_sub_sp_x->m_full[9];  // wqx
    tmp_x_hat_partial->m_full[6] = tmp_sub_sp_x->m_full[10]; // wqy
    tmp_x_hat_partial->m_full[7] = tmp_sub_sp_x->m_full[11]; // wqz

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

