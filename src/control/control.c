#include "control.h"
#include <stdlib.h>
#include <uquad_error_codes.h>

ctrl_t *control_init(void)
{
    ctrl_t *ctrl = (ctrl_t *)malloc(sizeof(ctrl_t));
    mem_alloc_check(ctrl);
    ctrl->K = uquad_mat_alloc(4,7);
    if(ctrl->K == NULL)
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

int control(ctrl_t *ctrl, uquad_mat_t *w, uquad_mat_t *x, sp_t *sp)
{
    int retval;
    if(sp != NULL)
    {
	err_check(ERROR_FAIL,"Not implemented! Should be NULL!");
    }
    err_check(ERROR_FAIL,"Not implemented!");
}

void control_deinit(ctrl_t *ctrl)
{
    if (ctrl == NULL)
    {
	err_log("No memory had been allocated for ctrl");
	return;
    }
    if(ctrl->K != NULL)
	uquad_mat_free(ctrl->K);
    free(ctrl);
}

