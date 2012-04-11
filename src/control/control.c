#include "control.h"
#include <stdlib.h>
#include <uquad_error_codes.h>

uquad_mat_t *tmp_sub_sp_x;
uquad_mat_t *tmp_x_hat_partial;
uquad_mat_t *w_tmp;

#if CTRL_INTEGRAL
int control_clear_int(ctrl_t * ctrl)
{
    int retval;
    retval = uquad_mat_zeros(ctrl->x_int);
    err_propagate(retval);
    return retval;
}
#endif

ctrl_t *control_init(void)
{
    ctrl_t *ctrl = (ctrl_t *)malloc(sizeof(ctrl_t));
    mem_alloc_check(ctrl);

    ctrl->K = uquad_mat_alloc(4,STATES_CONTROLLED);
    tmp_sub_sp_x = uquad_mat_alloc(12,1);
    w_tmp = uquad_mat_alloc(4,1);
    tmp_x_hat_partial = uquad_mat_alloc(STATES_CONTROLLED,1);
    if(ctrl->K == NULL || tmp_sub_sp_x == NULL ||
       tmp_x_hat_partial == NULL || w_tmp == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate aux mem!");
    }


#if CTRL_INTEGRAL
    int retval;
    ctrl->K_int = uquad_mat_alloc(4,STATES_CONTROLLED);
    ctrl->x_int = uquad_mat_alloc(STATES_CONTROLLED,1);
    if(ctrl->x_int == NULL || ctrl->K_int == NULL)
    {
	cleanup_log_if(ERROR_MALLOC,"Failed to allocate aux mem!");
    }
    retval = control_clear_int(ctrl);
    cleanup_log_if(retval, "Failed to clear integral term!");

    ctrl -> K_int -> m[0][0] = 4.96242080000000029827;
    ctrl -> K_int -> m[0][1] = 0.00000000000000000019;
    ctrl -> K_int -> m[0][2] = 0.00000000000001186735;
    ctrl -> K_int -> m[0][3] = -4.73642740000000017631;
    ctrl -> K_int -> m[0][4] = 0.00000000000071964657;
    ctrl -> K_int -> m[0][5] = 0.00000000000000000000;
    ctrl -> K_int -> m[0][6] = 0.00000000000003197442;
    ctrl -> K_int -> m[0][7] = -0.02617755600000000113;

    ctrl -> K_int -> m[1][0] = 4.96242080000000029827;
    ctrl -> K_int -> m[1][1] = -0.00000000000000018372;
    ctrl -> K_int -> m[1][2] = 0.00000000000000000000;
    ctrl -> K_int -> m[1][3] = 4.73642740000000017631;
    ctrl -> K_int -> m[1][4] = 0.00000000000071964656;
    ctrl -> K_int -> m[1][5] = 0.00000000000001953992;
    ctrl -> K_int -> m[1][6] = 0.00000000000000000000;
    ctrl -> K_int -> m[1][7] = 0.02617755600000000113;

    ctrl -> K_int -> m[2][0] = 4.96242080000000029827;
    ctrl -> K_int -> m[2][1] = 0.00000000000000000019;
    ctrl -> K_int -> m[2][2] = -0.00000000000001187434;
    ctrl -> K_int -> m[2][3] = -4.73642740000000017631;
    ctrl -> K_int -> m[2][4] = 0.00000000000071964656;
    ctrl -> K_int -> m[2][5] = 0.00000000000000000000;
    ctrl -> K_int -> m[2][6] = -0.00000000000003197442;
    ctrl -> K_int -> m[2][7] = -0.02617755600000000113;

    ctrl -> K_int -> m[3][0] = 4.96242080000000029827;
    ctrl -> K_int -> m[3][1] = 0.00000000000000018372;
    ctrl -> K_int -> m[3][2] = 0.00000000000000000001;
    ctrl -> K_int -> m[3][3] = 4.73642740000000017631;
    ctrl -> K_int -> m[3][4] = 0.00000000000071964657;
    ctrl -> K_int -> m[3][5] = -0.00000000000001953992;
    ctrl -> K_int -> m[3][6] = -0.00000000000000000000;
    ctrl -> K_int -> m[3][7] = 0.02617755600000000113;
#endif

    // GOOD MATRIX ?

    ctrl -> K -> m[0][0] = 4.97765931181421006357;
    ctrl -> K -> m[0][1] = -0.00000000000287443977;
    ctrl -> K -> m[0][2] = -204.60579720270845882624;
    ctrl -> K -> m[0][3] = -152.26264757055719201162;
    ctrl -> K -> m[0][4] = 13.04143342168293884242;
    ctrl -> K -> m[0][5] = 0.00000000000001692230;
    ctrl -> K -> m[0][6] = -25.72919728799993777102;
    ctrl -> K -> m[0][7] = -41.70433960553605601262;
    ctrl -> K -> m[1][0] = 4.97765931181420295815;
    ctrl -> K -> m[1][1] = 204.60579720270914094726;
    ctrl -> K -> m[1][2] = 0.00000000000472012680;
    ctrl -> K -> m[1][3] = 152.26264757056330267915;
    ctrl -> K -> m[1][4] = 13.04143342168256936020;
    ctrl -> K -> m[1][5] = 25.72919728799993066559;
    ctrl -> K -> m[1][6] = 0.00000000000030345749;
    ctrl -> K -> m[1][7] = 41.70433960553364016732;
    ctrl -> K -> m[2][0] = 4.97765931181459286847;
    ctrl -> K -> m[2][1] = -0.00000000000353261046;
    ctrl -> K -> m[2][2] = 204.60579720270226289358;
    ctrl -> K -> m[2][3] = -152.26264757057256815642;
    ctrl -> K -> m[2][4] = 13.04143342168285535365;
    ctrl -> K -> m[2][5] = 0.00000000000052652057;
    ctrl -> K -> m[2][6] = 25.72919728799985250589;
    ctrl -> K -> m[2][7] = -41.70433960553575047925;
    ctrl -> K -> m[3][0] = 4.97765931181445697717;
    ctrl -> K -> m[3][1] = -204.60579720270240500213;
    ctrl -> K -> m[3][2] = 0.00000000000108168787;
    ctrl -> K -> m[3][3] = 152.26264757056611642838;
    ctrl -> K -> m[3][4] = 13.04143342168257646563;
    ctrl -> K -> m[3][5] = -25.72919728800047067807;
    ctrl -> K -> m[3][6] = -0.00000000000022757349;
    ctrl -> K -> m[3][7] = 41.70433960553793895087;


    // LIGHT MATRIX
    /*
    ctl -> K -> m[0][0] = 0.49934047680803017588;
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
    */

    return ctrl;

    cleanup:
    control_deinit(ctrl);
    return NULL;
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

#if CTRL_INTEGRAL
    retval = uquad_mat_add(ctrl->x_int, ctrl->x_int, tmp_x_hat_partial);
    err_propagate(retval);
    retval = uquad_mat_prod(w_tmp, ctrl->K_int, ctrl->x_int);
    err_propagate(retval);
    retval = uquad_mat_add(w, w, w_tmp);
    err_propagate(retval);
#endif

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
    uquad_mat_free(w_tmp);
#if CTRL_INTEGRAL
    uquad_mat_free(ctrl->K_int);
    uquad_mat_free(ctrl->x_int);
#endif
    free(ctrl);
}

