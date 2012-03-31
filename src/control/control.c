#include "control.h"
#include <stdlib.h>
#include <uquad_error_codes.h>
#include <mot_control.h>

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

    ctrl -> K -> m[0][0] = 4.97765931181205889544;
    ctrl -> K -> m[0][1] = 0.00000000000451420227;
    ctrl -> K -> m[0][2] = -146.58502148335858805694;
    ctrl -> K -> m[0][3] = 13.04143342168014640947;
    ctrl -> K -> m[0][4] = 0.00000000000106279309;
    ctrl -> K -> m[0][5] = -22.07375211328743702666;
    ctrl -> K -> m[0][6] = 4.97789980888127647773;
    
    ctrl -> K -> m[1][0] = 4.97765931181207310630;
    ctrl -> K -> m[1][1] = 146.58502148335972492532;
    ctrl -> K -> m[1][2] = -0.00000000000327613045;
    ctrl -> K -> m[1][3] = 13.04143342168025121453;
    ctrl -> K -> m[1][4] = 22.07375211328694319945;
    ctrl -> K -> m[1][5] = -0.00000000000107384049;
    ctrl -> K -> m[1][6] = -4.97789980888144611981;
    
    ctrl -> K -> m[2][0] = 4.97765931181237153424;
    ctrl -> K -> m[2][1] = -0.00000000000473998584;
    ctrl -> K -> m[2][2] = 146.58502148335875858720;
    ctrl -> K -> m[2][3] = 13.04143342168074859444;
    ctrl -> K -> m[2][4] = -0.00000000000107944321;
    ctrl -> K -> m[2][5] = 22.07375211328751873907;
    ctrl -> K -> m[2][6] = 4.97789980888164773631;
    
    ctrl -> K -> m[3][0] = 4.97765931181217169410;
    ctrl -> K -> m[3][1] = -146.58502148335989545558;
    ctrl -> K -> m[3][2] = 0.00000000000307515036;
    ctrl -> K -> m[3][3] = 13.04143342168030450523;
    ctrl -> K -> m[3][4] = -22.07375211328693609403;
    ctrl -> K -> m[3][5] = 0.00000000000105388627;
    ctrl -> K -> m[3][6] = -4.97789980888135019654;


    /*
      ctrl -> K -> m[0][0] = 0.49934047680831794569;
      ctrl -> K -> m[0][1] = 0.00000000000003744260;
      ctrl -> K -> m[0][2] = -21.75937550244218243733;
      ctrl -> K -> m[0][3] = 3.85038861888382433207;
      ctrl -> K -> m[0][4] = 0.00000000000003921728;
      ctrl -> K -> m[0][5] = -8.15001366200123378292;
      ctrl -> K -> m[0][6] = 0.49977826290407101339;
      ctrl -> K -> m[1][0] = 0.49934047680834670047;
      ctrl -> K -> m[1][1] = 21.75937550244205809236;
      ctrl -> K -> m[1][2] = -0.00000000000010364819;
      ctrl -> K -> m[1][3] = 3.85038861888383676657;
      ctrl -> K -> m[1][4] = 8.15001366200116983407;
      ctrl -> K -> m[1][5] = -0.00000000000003326558;
      ctrl -> K -> m[1][6] = -0.49977826290402943554;
      ctrl -> K -> m[2][0] = 0.49934047680831950000;
      ctrl -> K -> m[2][1] = 0.00000000000001721972;
      ctrl -> K -> m[2][2] = 21.75937550244223572804;
      ctrl -> K -> m[2][3] = 3.85038861888395356203;
      ctrl -> K -> m[2][4] = -0.00000000000001767908;
      ctrl -> K -> m[2][5] = 8.15001366200125332284;
      ctrl -> K -> m[2][6] = 0.49977826290418553290;
      ctrl -> K -> m[3][0] = 0.49934047680834414695;
      ctrl -> K -> m[3][1] = -21.75937550244203322336;
      ctrl -> K -> m[3][2] = 0.00000000000009935126;
      ctrl -> K -> m[3][3] = 3.85038861888365513408;
      ctrl -> K -> m[3][4] = -8.15001366200116983407;
      ctrl -> K -> m[3][5] = 0.00000000000002454443;
      ctrl -> K -> m[3][6] = -0.49977826290411730970;
    */


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
    tmp_x_hat_partial->m_full[3] = tmp_sub_sp_x->m_full[8];  // vqz
    tmp_x_hat_partial->m_full[4] = tmp_sub_sp_x->m_full[9];  // wqx
    tmp_x_hat_partial->m_full[5] = tmp_sub_sp_x->m_full[10]; // wqy
    tmp_x_hat_partial->m_full[6] = tmp_sub_sp_x->m_full[11]; // wqz

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

