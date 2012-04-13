#include <math.h>
#include "uquad_kalman_gps.h"
#include <uquad_types.h>

// Auxiliaries for h
uquad_mat_t* Rv_gps = NULL;
uquad_mat_t* v_gps  = NULL;
uquad_mat_t* R_gps  = NULL;

// Auxiliaries for Update
uquad_mat_t* fx_gps = NULL;
uquad_mat_t* Fk_1_gps = NULL;
uquad_mat_t* Fk_1_T_gps = NULL;
uquad_mat_t* mtmp_gps = NULL;
uquad_mat_t* P__gps = NULL;

// Auxiliaries for Update
uquad_mat_t* hx_gps = NULL;
uquad_mat_t* z_gps = NULL;
uquad_mat_t* yk_gps = NULL;
uquad_mat_t* Hx_gps = NULL;
uquad_mat_t* HP__gps  = NULL;
uquad_mat_t* HT_gps   = NULL;
uquad_mat_t* HP_H_gps = NULL;
uquad_mat_t* Sk_gps   = NULL;
uquad_mat_t* Sk_1_gps = NULL;
uquad_mat_t* P_HT_gps = NULL;
uquad_mat_t* Kk_gps   = NULL;
uquad_mat_t* Kkyk_gps = NULL;
uquad_mat_t* x_hat_gps = NULL;
uquad_mat_t* I_gps    = NULL;
uquad_mat_t* KkH_gps  = NULL;
uquad_mat_t* IKH_gps  = NULL;

int f_gps(uquad_mat_t* fx, kalman_io_t* kalman_io_data)
{
    fx->m_full[0]  = kalman_io_data -> x_hat -> m_full[0];
    fx->m_full[1]  = kalman_io_data -> x_hat -> m_full[1];
    fx->m_full[2]  = kalman_io_data -> x_hat -> m_full[2];
    fx->m_full[3]  = kalman_io_data -> x_hat -> m_full[6];
    fx->m_full[4]  = kalman_io_data -> x_hat -> m_full[7];
    fx->m_full[5]  = kalman_io_data -> x_hat -> m_full[8];
    return ERROR_OK;
}

int h_gps(uquad_mat_t* hx, kalman_io_t* kalman_io_data)
{
    int retval;
    if(Rv==NULL)
    {
	Rv     = uquad_mat_alloc(3,1);
	v      = uquad_mat_alloc(3,1);
	R      = uquad_mat_alloc(3,3);
    }
    v -> m_full[0] = kalman_io_data -> x_hat -> m_full[6];
    v -> m_full[1] = kalman_io_data -> x_hat -> m_full[7];
    v -> m_full[2] = kalman_io_data -> x_hat -> m_full[8];
    retval = uquad_mat_rotate(false,Rv, v, 
			      kalman_io_data -> x_hat -> m_full[3],
			      kalman_io_data -> x_hat -> m_full[4],
			      kalman_io_data -> x_hat -> m_full[5],
			      R);
    err_propagate(retval);

    hx->m_full[0]  = kalman_io_data -> x_hat -> m_full[0];
    hx->m_full[1]  = kalman_io_data -> x_hat -> m_full[1];
    hx->m_full[2]  = kalman_io_data -> x_hat -> m_full[2];
    hx->m_full[3]  = Rv -> m_full[0];
    hx->m_full[4]  = Rv -> m_full[1];
    hx->m_full[5]  = Rv -> m_full[2];

    return ERROR_OK;
}

int F_gps(uquad_mat_t* Fx)
{
    int retval;
    retval = uquad_mat_eye(Fx);
    err_propagate(retval);
}

int H_gps(uquad_mat_t* Hx, kalman_io_t* kalman_io_data)
{
    double psi   = kalman_io_data -> x_hat -> m_full[3];
    double phi   = kalman_io_data -> x_hat -> m_full[4];
    double theta = kalman_io_data -> x_hat -> m_full[5];
    Hx->m[0][0] = 1;
    Hx->m[0][1] = 0;
    Hx->m[0][2] = 0;
    Hx->m[0][3] = 0;
    Hx->m[0][4] = 0;
    Hx->m[0][5] = 0;
    Hx->m[1][0] = 0;
    Hx->m[1][1] = 1;
    Hx->m[1][2] = 0;
    Hx->m[1][3] = 0;
    Hx->m[1][4] = 0;
    Hx->m[1][5] = 0;
    Hx->m[2][0] = 0;
    Hx->m[2][1] = 0;
    Hx->m[2][2] = 1;
    Hx->m[2][3] = 0;
    Hx->m[2][4] = 0;
    Hx->m[2][5] = 0;
    Hx->m[3][0] = 0;
    Hx->m[3][1] = 0;
    Hx->m[3][2] = 0;
    Hx->m[3][3] = cos(phi)*cos(theta);
    Hx->m[3][4] = cos(theta)*sin(phi)*sin(psi) - cos(psi)*sin(theta);
    Hx->m[3][5] = sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi);
    Hx->m[4][0] = 0;
    Hx->m[4][1] = 0;
    Hx->m[4][2] = 0;
    Hx->m[4][3] = cos(phi)*sin(theta);
    Hx->m[4][4] = cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta);
    Hx->m[4][5] = cos(psi)*sin(phi)*sin(theta) - cos(theta)*sin(psi);
    Hx->m[5][0] = 0;
    Hx->m[5][1] = 0;
    Hx->m[5][2] = 0;
    Hx->m[5][3] = -sin(phi);
    Hx->m[5][4] = cos(phi)*sin(psi);
    Hx->m[5][5] = cos(phi)*cos(psi);
    return ERROR_OK;
}

int uquad_kalman_gps(kalman_io_t* kalman_io_data, gps_data_t* gps_i_data)
{
    int retval;
    if(Fk_1==NULL)
    {
	// Auxiliaries for prediction
	fx_gps     = uquad_mat_alloc(6,1);
	Fk_1_gps   = uquad_mat_alloc(6,6);
	Fk_1_T_gps = uquad_mat_alloc(6,6);
	mtmp_gps   = uquad_mat_alloc(6,6);
	P__gps     = uquad_mat_alloc(6,6);

	// Auxiliaries for update
	hx_gps        = uquad_mat_alloc(6,1);
	z_gps         = uquad_mat_alloc(6,1);
	yk_gps        = uquad_mat_alloc(6,1);
	Hx_gps        = uquad_mat_alloc(6,6);
	HP__gps       = uquad_mat_alloc(6,6);
	HT_gps        = uquad_mat_alloc(6,6);
	HP_H_gps      = uquad_mat_alloc(6,6);
	Sk_gps        = uquad_mat_alloc(6,6);
	Sk_1_gps      = uquad_mat_alloc(6,6);
	P_HT_gps      = uquad_mat_alloc(6,6);
	Kk_gps        = uquad_mat_alloc(6,6);
	Kkyk_gps      = uquad_mat_alloc(6,1);
	x_hat_gps     = uquad_mat_alloc(6,1);
	I_gps         = uquad_mat_alloc(6,6);
	KkH_gps       = uquad_mat_alloc(6,6);
	IKH_gps       = uquad_mat_alloc(6,6);
    }

    // Prediction
    retval = f_gps(fx_gps, kalman_io_data);
    err_propagate(retval);
    retval = F_gps(Fk_1_gps);
    err_propagate(retval);
    retval = uquad_mat_transpose(Fk_1_T_gps, Fk_1_gps);
    err_propagate(retval);
    retval = uquad_mat_prod(mtmp_gps, Fk_1_gps, kalman_io_data -> P_gps);
    err_propagate(retval);
    retval = uquad_mat_prod(Fk_1_gps,mtmp_gps, Fk_1_T_gps); // Aca lo vuelvo a guardar en Fk_1 para no hacer otra variable temporal
    err_propagate(retval);
    retval = uquad_mat_add(P__gps,Fk_1_gps,kalman_io_data->Q_gps);
    err_propagate(retval);

    // Update
    retval = h_gps(hx_gps, kalman_io_data);
    err_propagate(retval);

    z_gps -> m_full[0] = gps_i_data -> pos -> m_full[0];
    z_gps -> m_full[1] = gps_i_data -> pos -> m_full[1];
    z_gps -> m_full[2] = gps_i_data -> pos -> m_full[2];
    z_gps -> m_full[3] = gps_i_data -> vel -> m_full[0];
    z_gps -> m_full[4] = gps_i_data -> vel -> m_full[1];
    z_gps -> m_full[5] = gps_i_data -> vel -> m_full[2];

    retval =  uquad_mat_sub(yk_gps, z_gps, hx_gps);
    err_propagate(retval);
    retval = H_gps(Hx_gps, kalman_io_data);
    err_propagate(retval);
    retval = uquad_mat_prod(HP__gps,Hx_gps,P__gps);
    err_propagate(retval);
    retval = uquad_mat_transpose(HT_gps,Hx_gps);
    err_propagate(retval);
    retval = uquad_mat_prod(HP_H_gps,HP__gps,HT_gps);
    err_propagate(retval);
    retval = uquad_mat_add(Sk_gps,HP_H_gps,kalman_io_data -> R_gps);
    err_propagate(retval);
    retval = uquad_mat_inv(Sk_1_gps,Sk_gps,NULL,NULL);
    err_propagate(retval);
    retval = uquad_mat_prod(P_HT_gps,P__gps,HT_gps);
    err_propagate(retval);
    retval = uquad_mat_prod(Kk_gps,P_HT_gps,Sk_1_gps);
    err_propagate(retval);
    retval = uquad_mat_prod(Kkyk_gps,Kk_gps,yk_gps);
    err_propagate(retval);
    retval = uquad_mat_add(x_hat_gps, fx_gps, Kkyk_gps);
    err_propagate(retval);
    retval =  uquad_mat_eye(I_gps);
    err_propagate(retval);
    retval = uquad_mat_prod(KkH_gps, Kk_gps, Hx_gps);
    err_propagate(retval);
    retval = uquad_mat_sub(IKH_gps,I_gps,KkH_gps);
    err_propagate(retval);
    retval = uquad_mat_prod(kalman_io_data -> P_gps, IKH_gps, P__gps);
    err_propagate(retval);

    kalman_io_data -> x_hat -> m_full[0] = x_hat_gps -> m_full[0];
    kalman_io_data -> x_hat -> m_full[1] = x_hat_gps -> m_full[1];
    kalman_io_data -> x_hat -> m_full[2] = x_hat_gps -> m_full[2];
    kalman_io_data -> x_hat -> m_full[6] = x_hat_gps -> m_full[3];
    kalman_io_data -> x_hat -> m_full[7] = x_hat_gps -> m_full[4];
    kalman_io_data -> x_hat -> m_full[8] = x_hat_gps -> m_full[5];

    return ERROR_OK;
}

void kalman_gps_deinit(kalman_io_t *kalman_io_data)
{
    // Auxiliaries for h
    uquad_mat_free(Rv_gps);
    uquad_mat_free(v_gps );
    uquad_mat_free(R_gps );

    // Auxiliaries for Update
    uquad_mat_free(fx_gps);
    uquad_mat_free(Fk_1_gps);
    uquad_mat_free(Fk_1_T_gps);
    uquad_mat_free(mtmp_gps);
    uquad_mat_free(P__gps);

    // Auxiliaries for Update
    uquad_mat_free(hx_gps);
    uquad_mat_free(z_gps);
    uquad_mat_free(yk_gps);
    uquad_mat_free(Hx_gps);
    uquad_mat_free(HP__gps);
    uquad_mat_free(HT_gps  );
    uquad_mat_free(HP_H_gps);
    uquad_mat_free(Sk_gps  );
    uquad_mat_free(Sk_1_gps);
    uquad_mat_free(P_HT_gps);
    uquad_mat_free(Kk_gps  );
    uquad_mat_free(Kkyk_gps);
    uquad_mat_free(x_hat_gps);
    uquad_mat_free(I_gps);
    uquad_mat_free(KkH_gps);
    uquad_mat_free(IKH_gps);

    if(kalman_io_data != NULL)
    {
	uquad_mat_free(kalman_io_data->x_hat);
	uquad_mat_free(kalman_io_data->x_);
	uquad_mat_free(kalman_io_data->u);
	uquad_mat_free(kalman_io_data->z);
	uquad_mat_free(kalman_io_data->Q);
	uquad_mat_free(kalman_io_data->R);
	uquad_mat_free(kalman_io_data->P);
	uquad_mat_free(kalman_io_data->Q_gps);
	uquad_mat_free(kalman_io_data->R_gps);
	uquad_mat_free(kalman_io_data->P_gps);
	free(kalman_io_data);
    }
}
