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

    cctl -> K -> m[0][0] = 0.49934047680803017588;
ctl -> K -> m[0][1] = 0.00000000000017122169;
ctl -> K -> m[0][2] = -21.75937550244206519778;
ctl -> K -> m[0][3] = -15.62508835442807964000;
ctl -> K -> m[0][4] = 3.85038861888411654277;
ctl -> K -> m[0][5] = 0.00000000000001624196;
ctl -> K -> m[0][6] = -8.15001366200120891392;
ctl -> K -> m[0][7] = -13.27952808741211931931;

ctl -> K -> m[1][0] = 0.49934047680804971581;
ctl -> K -> m[1][1] = 21.75937550244172768998;
ctl -> K -> m[1][2] = -0.00000000000003756029;
ctl -> K -> m[1][3] = 15.62508835442773680313;
ctl -> K -> m[1][4] = 3.85038861888415251400;
ctl -> K -> m[1][5] = 8.15001366200113785965;
ctl -> K -> m[1][6] = 0.00000000000001882213;
ctl -> K -> m[1][7] = 13.27952808741187418207;

ctl -> K -> m[2][0] = 0.49934047680801718627;
ctl -> K -> m[2][1] = -0.00000000000008256758;
ctl -> K -> m[2][2] = 21.75937550244189822024;
ctl -> K -> m[2][3] = -15.62508835442801213844;
ctl -> K -> m[2][4] = 3.85038861888394690070;
ctl -> K -> m[2][5] = 0.00000000000000081247;
ctl -> K -> m[2][6] = 8.15001366200113785965;
ctl -> K -> m[2][7] = -13.27952808741166634832;

ctl -> K -> m[3][0] = 0.49934047680805709879;
ctl -> K -> m[3][1] = -21.75937550244187335124;
ctl -> K -> m[3][2] = 0.00000000000007485393;
ctl -> K -> m[3][3] = 15.62508835442848997843;
ctl -> K -> m[3][4] = 3.85038861888423955548;
ctl -> K -> m[3][5] = -8.15001366200116628136;
ctl -> K -> m[3][6] = 0.00000000000003355322;
ctl -> K -> m[3][7] = 13.27952808741203050147;

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

