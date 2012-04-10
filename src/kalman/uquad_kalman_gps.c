#include <math.h>
#include "uquad_kalman.h"
#include <uquad_types.h>

uquad_mat_t* Fk_1 = NULL;
uquad_mat_t* Fk_1_T = NULL;
uquad_mat_t* tmp = NULL;
uquad_mat_t* tmp2 = NULL;
uquad_mat_t* mtmp = NULL;
uquad_mat_t* P_ = NULL;
uquad_mat_t* w2 = NULL;
uquad_mat_t* TM = NULL;
uquad_mat_t* D = NULL;
uquad_mat_t* H = NULL;

uquad_mat_t* fx = NULL;
uquad_mat_t* hx = NULL;
uquad_mat_t* Fx = NULL;
uquad_mat_t* Hx = NULL;
uquad_mat_t* Rx = NULL;
uquad_mat_t* Ry = NULL;
uquad_mat_t* Rz = NULL;
uquad_mat_t* RzRy = NULL;
uquad_mat_t* RzRyRx = NULL;
uquad_mat_t* Rv = NULL;
uquad_mat_t* v = NULL;

// Auxiliares Update
uquad_mat_t* yk = NULL;
uquad_mat_t* HT   = NULL;
uquad_mat_t* HP_  = NULL;
uquad_mat_t* HP_H = NULL;
uquad_mat_t* Sk   = NULL;
uquad_mat_t* P_HT = NULL;
uquad_mat_t* Kk   = NULL;
uquad_mat_t* Kkyk = NULL;
uquad_mat_t* KkH  = NULL;
uquad_mat_t* IKH  = NULL;
uquad_mat_t* Sk_1 = NULL;
uquad_mat_t* I    = NULL;

int Rx(uquad_mat_t* Rx, float psi)
{
    Rx -> m[0][0] = 1.00000000000000000000;
    Rx -> m[0][1] = 0.00000000000000000000;
    Rx -> m[0][2] = 0.00000000000000000000;
    Rx -> m[1][0] = 0.00000000000000000000;
    Rx -> m[1][1] = cos(psi);
    Rx -> m[1][2] = -sin(psi);
    Rx -> m[2][0] = 0.00000000000000000000;
    Rx -> m[2][1] = sin(psi);
    Rx -> m[2][2] = cos(psi);
    return ERROR_OK;
}

int Ry(uquad_mat_t* Ry, float phi)
{
    Ry -> m[0][0] = cos(phi);
    Ry -> m[0][1] = 0.00000000000000000000;
    Ry -> m[0][2] = sin(phi);
    Ry -> m[1][0] = 0.00000000000000000000;
    Ry -> m[1][1] = 1.00000000000000000000;
    Ry -> m[1][2] = 0.00000000000000000000;
    Ry -> m[2][0] = -sin(phi);
    Ry -> m[2][1] = 0.00000000000000000000;
    Ry -> m[2][2] = cos(phi);
    return ERROR_OK;
}

int Rz(uquad_mat_t* Rz, float theta)
{
    Rz -> m[0][0] = cos(theta);
    Rz -> m[0][1] = -sin(theta);
    Rz -> m[0][2] = 0.00000000000000000000;
    Rz -> m[1][0] = sin(theta);
    Rz -> m[1][1] = cos(theta);
    Rz -> m[1][2] = 0.00000000000000000000;
    Rz -> m[2][0] = 0.00000000000000000000;
    Rz -> m[2][1] = 0.00000000000000000000;
    Rz -> m[2][2] = 1.00000000000000000000;
    return ERROR_OK;
}

int f(uquad_mat_t* fx, kalman_io_t* kalman_io_data)
{
    fx->m_full[0]  = kalman_io_data -> x_hat -> m_full[0];
    fx->m_full[1]  = kalman_io_data -> x_hat -> m_full[1];
    fx->m_full[2]  = kalman_io_data -> x_hat -> m_full[2];
    fx->m_full[3]  = kalman_io_data -> x_hat -> m_full[6];
    fx->m_full[4]  = kalman_io_data -> x_hat -> m_full[7];
    fx->m_full[5]  = kalman_io_data -> x_hat -> m_full[8];
    return ERROR_OK;
}

int h(uquad_mat_t* hx, kalman_io_t* kalman_io_data)
{
    int retval;
    if(Rx==NULL)
    {
	Rx     = uquad_mat_alloc(3,3);
	Ry     = uquad_mat_alloc(3,3);
	Rz     = uquad_mat_alloc(3,3);
	RzRy   = uquad_mat_alloc(3,3);
	RzRyRx = uquad_mat_alloc(3,3);
	Rv     = uquad_mat_alloc(3,1);
	v      = uquad_mat_alloc(3,1);
    }
    retval = Rx(Rx, kalman_io_data -> x_hat -> m_full[3]);
    err_propagate(retval);
    retval = Ry(Ry, kalman_io_data -> x_hat -> m_full[4]);
    err_propagate(retval);
    retval = Rz(Rz, kalman_io_data -> x_hat -> m_full[5]);
    err_propagate(retval);
    retval = uquad_mat_prod(RzRy, Rz, Ry);
    err_propagate(retval);
    retval = uquad_mat_prod(RzRyRx, RzRy, Rx);
    err_propagate(retval);
    v -> m_full[0] = kalman_io_data -> x_hat -> m_full[6];
    v -> m_full[1] = kalman_io_data -> x_hat -> m_full[7];
    v -> m_full[2] = kalman_io_data -> x_hat -> m_full[8];
    retval = uquad_mat_prod(Rv, RzRyRx, v);
    err_propagate(retval);

    hx->m_full[0]  = kalman_io_data -> x_hat -> m_full[0];
    hx->m_full[1]  = kalman_io_data -> x_hat -> m_full[1];
    hx->m_full[2]  = kalman_io_data -> x_hat -> m_full[2];
    hx->m_full[3]  = Rv -> m_full[0];
    hx->m_full[4]  = Rv -> m_full[1];
    hx->m_full[5]  = Rv -> m_full[2];

    return ERROR_OK;
}

int F(uquad_mat_t* Fx)
{
    int retval;
    retval = uquad_mat_eye(Fx);
    err_propagate(retval);
}

int H(uquad_mat_t* Hx, kalman_io_t* kalman_io_data)
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

// TODO arreglar de aca pa abajo

int uquad_kalman(kalman_io_t * kalman_io_data, data del GPS)
{
    int retval;
    if(Fk_1==NULL)
    {
	Fk_1   = uquad_mat_alloc(12,12);
	Fk_1_T = uquad_mat_alloc(12,12);
	mtmp   = uquad_mat_alloc(12,12);
	P_     = uquad_mat_alloc(12,12);      
	retval = H_init();
	err_propagate(retval);
	Fx = uquad_mat_alloc(12,12);
	fx = uquad_mat_alloc(12,1);
	hx = uquad_mat_alloc(10,1);

	// Auxiliares para el update
	yk = uquad_mat_alloc(10,1);
	HT   = uquad_mat_alloc(12,10);
	HP_  = uquad_mat_alloc(10,12);
	HP_H = uquad_mat_alloc(10,10);
	Sk   = uquad_mat_alloc(10,10);
	P_HT = uquad_mat_alloc(12,10);
	Kk   = uquad_mat_alloc(12,10);
	Kkyk = uquad_mat_alloc(12,1);
	KkH  = uquad_mat_alloc(12,12);
	IKH  = uquad_mat_alloc(12,12);
	Sk_1 = uquad_mat_alloc(10,10);
	I    = uquad_mat_alloc(12,12);
    }

    retval = store_data(kalman_io_data, w, data, T);
    err_propagate(retval);

    // Prediction
    retval = f(kalman_io_data -> x_, kalman_io_data);
    err_propagate(retval);
    retval = F(Fk_1, kalman_io_data);
    err_propagate(retval);
    retval = uquad_mat_transpose(Fk_1_T, Fk_1);
    err_propagate(retval);
    retval = uquad_mat_prod(mtmp, Fk_1, kalman_io_data->P);
    err_propagate(retval);
    retval = uquad_mat_prod(Fk_1,mtmp, Fk_1_T); // Aca lo vuelvo a guardar en Fk_1 para no hacer otra variable temporal
    err_propagate(retval);
    retval = uquad_mat_add(P_,Fk_1,kalman_io_data->Q);
    err_propagate(retval);

    // Update
    retval = h(hx, kalman_io_data);
    err_propagate(retval);
    retval =  uquad_mat_sub(yk, kalman_io_data -> z , hx);
    err_propagate(retval);
    retval = uquad_mat_prod(HP_,H,P_);
    err_propagate(retval);
    retval = uquad_mat_transpose(HT,H);
    err_propagate(retval);
    retval = uquad_mat_prod(HP_H,HP_,HT);
    err_propagate(retval);
    retval = uquad_mat_add(Sk,HP_H,kalman_io_data -> R); // Sk
    err_propagate(retval);
    retval = uquad_mat_inv(Sk_1,Sk,NULL,NULL);
    err_propagate(retval);
    retval = uquad_mat_prod(P_HT,P_,HT);
    err_propagate(retval);
    retval = uquad_mat_prod(Kk,P_HT,Sk_1);
    err_propagate(retval);
    retval = uquad_mat_prod(Kkyk,Kk,yk);
    err_propagate(retval);
    retval = uquad_mat_add(kalman_io_data->x_hat, kalman_io_data->x_, Kkyk);
    err_propagate(retval);
    retval =  uquad_mat_eye(I);
    err_propagate(retval);
    retval = uquad_mat_prod(KkH, Kk, H);
    err_propagate(retval);
    retval = uquad_mat_sub(IKH,I,KkH);
    err_propagate(retval);
    retval = uquad_mat_prod(kalman_io_data->P, IKH, P_);
    err_propagate(retval);

    return ERROR_OK;
}
