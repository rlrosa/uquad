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

int uquad_kalman(kalman_io_t * k_io_data, uquad_mat_t* w, imu_data_t* data)

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

    retval = store_data(k_io_data, w, data);
    err_propagate(retval);

    // Prediction
    retval = f(k_io_data -> x_, k_io_data);
    err_propagate(retval);
    retval = F(Fk_1, k_io_data);
    err_propagate(retval);
    retval = uquad_mat_transpose(Fk_1_T, Fk_1);
    err_propagate(retval);
    retval = uquad_mat_prod(mtmp, Fk_1, k_io_data->P);
    err_propagate(retval);
    retval = uquad_mat_prod(Fk_1,mtmp, Fk_1_T); // Aca lo vuelvo a guardar en Fk_1 para no hacer otra variable temporal
    err_propagate(retval);
    retval = uquad_mat_add(P_,Fk_1,k_io_data->Q);
    err_propagate(retval);

    // Update
    retval = h(hx, k_io_data);
    err_propagate(retval);
    retval =  uquad_mat_sub(yk, k_io_data -> z , hx);
    err_propagate(retval);
    retval = uquad_mat_prod(HP_,H,P_);
    err_propagate(retval);
    retval = uquad_mat_transpose(HT,H);
    err_propagate(retval);
    retval = uquad_mat_prod(HP_H,HP_,HT);
    err_propagate(retval);
    retval = uquad_mat_add(Sk,HP_H,k_io_data -> R); // Sk
    err_propagate(retval);
    retval = uquad_mat_inv(Sk_1,Sk,NULL,NULL);
    err_propagate(retval);
    retval = uquad_mat_prod(P_HT,P_,HT);
    err_propagate(retval);
    retval = uquad_mat_prod(Kk,P_HT,Sk_1);
    err_propagate(retval);
    retval = uquad_mat_prod(Kkyk,Kk,yk);
    err_propagate(retval);
    retval = uquad_mat_add(k_io_data->x_hat, k_io_data->x_, Kkyk);
    err_propagate(retval);
    retval =  uquad_mat_eye(I);
    err_propagate(retval);
    retval = uquad_mat_prod(KkH, Kk, H);
    err_propagate(retval);
    retval = uquad_mat_sub(IKH,I,KkH);
    err_propagate(retval);
    retval = uquad_mat_prod(k_io_data->P, IKH, P_);
    err_propagate(retval);
}

int drag(uquad_mat_t* drag, uquad_mat_t* w)
{  
    int retval;
    if(w2==NULL)
    {
	w2 = uquad_mat_alloc(4,1);
	tmp = uquad_mat_alloc(4,1);
	tmp2 = uquad_mat_alloc(4,1);
    }
    double A1 = 0.0000034734;
    double A2 = -0.00013205;
    retval = uquad_mat_dot_product(w2,w,w);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(tmp, w2, A1);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(tmp2, w, A2);
    err_propagate(retval);
    retval = uquad_mat_add(drag,tmp,tmp2);
    err_propagate(retval);
}

int drive(uquad_mat_t* drive, uquad_mat_t* w)
{  
    int retval;
    if(w2==NULL)
    {
	w2 = uquad_mat_alloc(4,1);
	tmp = uquad_mat_alloc(4,1);
	tmp2 = uquad_mat_alloc(4,1);
    }
    double A1 = 0.000035296;
    double A2 = -0.00049293;
    retval = uquad_mat_dot_product(w2,w,w);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(tmp, w2, A1);
    err_propagate(retval);
    retval = uquad_mat_scalar_mul(tmp2, w, A2);
    err_propagate(retval);
    retval = uquad_mat_add(drive,tmp,tmp2);
    err_propagate(retval);
}

int f(uquad_mat_t* fx, kalman_io_t* k_io_data)
{
    double x     = k_io_data -> x_hat -> m_full[1];
    double y     = k_io_data -> x_hat -> m_full[1];
    double z     = k_io_data -> x_hat -> m_full[2];
    double psi   = k_io_data -> x_hat -> m_full[3];
    double phi   = k_io_data -> x_hat -> m_full[4];
    double theta = k_io_data -> x_hat -> m_full[5];
    double vqx   = k_io_data -> x_hat -> m_full[6];
    double vqy   = k_io_data -> x_hat -> m_full[7];
    double vqz   = k_io_data -> x_hat -> m_full[8];
    double wqx   = k_io_data -> x_hat -> m_full[9];
    double wqy   = k_io_data -> x_hat -> m_full[10];
    double wqz   = k_io_data -> x_hat -> m_full[11];
    double T     = k_io_data -> T;
  
    int retval;
    double* w    = k_io_data -> u -> m_full;
    uquad_mat_t* w_mat    = k_io_data -> u;
    if(TM == NULL)
    {
	TM = uquad_mat_alloc(4,1);
	D = uquad_mat_alloc(4,1);
    }
    retval =  drive(TM,w_mat);
    err_propagate(retval);
    retval =  drag(D,w_mat);
    err_propagate(retval);
    double* TM_vec = TM->m_full;
    double* D_vec = TM->m_full;

    fx->m_full[0]  = x     + T *(vqx*cos(phi)*cos(theta)+vqy*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vqz*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi)) ) ;
    fx->m_full[1]  = y     + T *(vqx*cos(phi)*sin(theta)+vqy*(sin(theta)*sin(phi)*sin(psi)+cos(psi)*cos(theta))+vqz*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi)) ) ;
    fx->m_full[2]  = z     + T *(-vqx*sin(phi)+vqy*cos(phi)*sin(psi)+vqz*cos(psi)*cos(psi));
    fx->m_full[3]  = psi   + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
    fx->m_full[4]  = phi   + T*( wqy*cos(psi)-wqz*sin(psi));
    fx->m_full[5]  = theta + T*( wqz*cos(psi)/cos(phi)+wqy*sin(psi)/cos(phi));
    fx->m_full[6]  = vqx   + T*( vqy*wqz-vqz*wqy+GRAVITY*sin(phi));
    fx->m_full[7]  = vqy   + T*( vqz*wqx-vqx*wqz-GRAVITY*cos(phi)*sin(psi));
    fx->m_full[8]  = vqz   + T*( vqx*wqy-vqy*wqx-GRAVITY*cos(phi)*cos(psi)+1/MASA*(TM_vec[0]+TM_vec[1]+TM_vec[2]+TM_vec[3]));
    fx->m_full[9] = wqx   + T*( wqy*wqz*(IYY-IZZ)+wqy*IZZM*(w[0]-w[1]+w[2]-w[3])+LENGTH*(TM_vec[1]-TM_vec[3]) )/IXX ;
    fx->m_full[10] = wqy   + T*( wqx*wqz*(IZZ-IXX)+wqx*IZZM*(w[0]-w[1]+w[2]-w[3])+LENGTH*(TM_vec[2]-TM_vec[0]) )/IYY;
    // fx->m_full[11] = wqz   + T*( -IZZM*(dw[0]-dw[1]+dw[2]-dw[3])+D[0]-D[1]+D[2]-D[3] )/IZZ;
    fx->m_full[11] = wqz   + T*( D_vec[0]-D_vec[1]+D_vec[2]-D_vec[3] )/IZZ;
}

int h(uquad_mat_t* hx, kalman_io_t* k_io_data)
{
    int retval;
    if(TM == NULL)
    {
	TM = uquad_mat_alloc(4,1);
	D = uquad_mat_alloc(4,1);
    }
    retval = drive(TM,k_io_data->u);
    err_propagate(retval);
    double* TM_vec = TM -> m_full;

    hx->m_full[0]  = k_io_data -> x_ -> m_full[3];
    hx->m_full[1]  = k_io_data -> x_ -> m_full[4];
    hx->m_full[2]  = k_io_data -> x_ -> m_full[5];
    hx->m_full[3]  = 0;
    hx->m_full[4]  = 0;
    hx->m_full[5]  = 1/MASA*(TM_vec[0]+TM_vec[1]+TM_vec[2]+TM_vec[3]);
    hx->m_full[6]  = k_io_data -> x_ -> m_full[9];
    hx->m_full[7]  = k_io_data -> x_ -> m_full[10];
    hx->m_full[8]  = k_io_data -> x_ -> m_full[11];
    hx->m_full[9] = k_io_data -> x_ -> m_full[2];
}

int F(uquad_mat_t* Fx, kalman_io_t* k_io_data)
{
    double x     = k_io_data -> x_hat -> m_full[0];
    double y     = k_io_data -> x_hat -> m_full[1];
    double z     = k_io_data -> x_hat -> m_full[2];
    double psi   = k_io_data -> x_hat -> m_full[3];
    double phi   = k_io_data -> x_hat -> m_full[4];
    double theta = k_io_data -> x_hat -> m_full[5];
    double vqx   = k_io_data -> x_hat -> m_full[6];
    double vqy   = k_io_data -> x_hat -> m_full[7];
    double vqz   = k_io_data -> x_hat -> m_full[8];
    double wqx   = k_io_data -> x_hat -> m_full[9];
    double wqy   = k_io_data -> x_hat -> m_full[10];
    double wqz   = k_io_data -> x_hat -> m_full[11];
    double T     = k_io_data -> T;

    double* w    = k_io_data -> u -> m_full;
    uquad_mat_t* w_t    = k_io_data -> u;
    int retval;

    if(TM == NULL)
    {
	TM = uquad_mat_alloc(4,1);
	D = uquad_mat_alloc(4,1);
    }

    retval =  drive(TM,w_t);
    err_propagate(retval);
    retval =  drag(D,w_t);
    err_propagate(retval);

    Fx->m[0][0] = 1;
    Fx->m[0][1] = 0;
    Fx->m[0][2] = 0;
    Fx->m[0][3] = T*(vqz*(cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi)) + vqy*cos(psi)*cos(theta)*sin(phi));
    Fx->m[0][4] = T*(vqy*(sin(phi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)) - vqx*cos(theta)*sin(phi) + vqz*cos(phi)*cos(psi)*cos(theta));
    Fx->m[0][5] = -T*(vqy*(cos(phi)*cos(theta) + sin(phi)*sin(psi)*sin(theta)) - vqz*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + vqx*cos(phi)*sin(theta));
    Fx->m[0][6] = T*cos(phi)*cos(theta);
    Fx->m[0][7] = -T*(cos(phi)*sin(theta) - cos(theta)*sin(phi)*sin(psi));
    Fx->m[0][8] = T*(sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi));
    Fx->m[0][9] = 0;
    Fx->m[0][10] = 0;
    Fx->m[0][11] = 0;

    Fx->m[1][0] = 0;
    Fx->m[1][1] = 1;
    Fx->m[1][2] = 0;
    Fx->m[1][3] = -T*(vqy*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + vqz*(cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta)));
    Fx->m[1][4] = T*(vqz*cos(phi)*cos(psi)*sin(theta) - vqx*sin(phi)*sin(theta) + vqy*cos(phi)*sin(psi)*sin(theta));
    Fx->m[1][5] = T*(vqz*(sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi)) - vqy*(cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi)) + vqx*cos(phi)*cos(theta));
    Fx->m[1][6] = T*cos(phi)*sin(theta);
    Fx->m[1][7] = T*(cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta));
    Fx->m[1][8] = -T*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta));
    Fx->m[1][9] = 0;
    Fx->m[1][10] = 0;
    Fx->m[1][11] = 0;

    Fx->m[2][0] = 0;
    Fx->m[2][1] = 0;
    Fx->m[2][2] = 1;
    Fx->m[2][3] = T*(vqy*cos(phi)*cos(psi) - 2*vqz*cos(psi)*sin(psi));
    Fx->m[2][4] = -T*(vqx*cos(phi) + vqy*sin(phi)*sin(psi));
    Fx->m[2][5] = 0;
    Fx->m[2][6] = -T*sin(phi);
    Fx->m[2][7] = T*cos(phi)*sin(psi);
    Fx->m[2][8] = T*uquad_square(cos(psi));
    Fx->m[2][9] = 0;
    Fx->m[2][10] = 0;
    Fx->m[2][11] = 0;

    Fx->m[3][0] = 0;
    Fx->m[3][1] = 0;
    Fx->m[3][2] = 0;
    Fx->m[3][3] = T*(wqy*cos(psi)*tan(phi) - wqz*sin(psi)*tan(phi)) + 1;
    Fx->m[3][4] = T*(wqz*cos(psi)*(uquad_square(tan(phi)) + 1) + wqy*sin(psi)*(uquad_square(tan(phi)) + 1));
    Fx->m[3][5] = 0;
    Fx->m[3][6] = 0;
    Fx->m[3][7] = 0;
    Fx->m[3][8] = 0;
    Fx->m[3][9] = T;
    Fx->m[3][10] = T*sin(psi)*tan(phi);
    Fx->m[3][11] = T*cos(psi)*tan(phi);

    Fx->m[4][0] = 0;
    Fx->m[4][1] = 0;
    Fx->m[4][2] = 0;
    Fx->m[4][3] = -T*(wqz*cos(psi) + wqy*sin(psi));
    Fx->m[4][4] = 1;
    Fx->m[4][5] = 0;
    Fx->m[4][6] = 0;
    Fx->m[4][7] = 0;
    Fx->m[4][8] = 0;
    Fx->m[4][9] = 0;
    Fx->m[4][10] = T*cos(psi);
    Fx->m[4][11] = -T*sin(psi);

    Fx->m[5][0] = 0;
    Fx->m[5][1] = 0;
    Fx->m[5][2] = 0;
    Fx->m[5][3] = T*((wqy*cos(psi))/cos(phi) - (wqz*sin(psi))/cos(phi));
    Fx->m[5][4] = T*((wqz*cos(psi)*sin(phi))/uquad_square(cos(phi)) + (wqy*sin(phi)*sin(psi))/uquad_square(cos(phi)));
    Fx->m[5][5] = 1;
    Fx->m[5][6] = 0;
    Fx->m[5][7] = 0;
    Fx->m[5][8] = 0;
    Fx->m[5][9] = 0;
    Fx->m[5][10] = (T*sin(psi))/cos(phi);
    Fx->m[5][11] = (T*cos(psi))/cos(phi);

    Fx->m[6][0] = 0;
    Fx->m[6][1] = 0;
    Fx->m[6][2] = 0;
    Fx->m[6][3] = 0;
    Fx->m[6][4] = T*GRAVITY*cos(phi);
    Fx->m[6][5] = 0;
    Fx->m[6][6] = 1;
    Fx->m[6][7] = T*wqz;
    Fx->m[6][8] = -T*wqy;
    Fx->m[6][9] = 0;
    Fx->m[6][10] = -T*vqz;
    Fx->m[6][11] = T*vqy;

    Fx->m[7][0] = 0;
    Fx->m[7][1] = 0;
    Fx->m[7][2] = 0;
    Fx->m[7][3] = -T*GRAVITY*cos(phi)*cos(psi);
    Fx->m[7][4] = T*GRAVITY*sin(phi)*sin(psi);
    Fx->m[7][5] = 0;
    Fx->m[7][6] = -T*wqz;
    Fx->m[7][7] = 1;
    Fx->m[7][8] = T*wqx;
    Fx->m[7][9] = T*vqz;
    Fx->m[7][10] = 0;
    Fx->m[7][11] = -T*vqx;

    Fx->m[8][0] = 0;
    Fx->m[8][1] = 0;
    Fx->m[8][2] = 0;
    Fx->m[8][3] = T*GRAVITY*cos(phi)*sin(psi);
    Fx->m[8][4] = T*GRAVITY*cos(psi)*sin(phi);
    Fx->m[8][5] = 0;
    Fx->m[8][6] = T*wqy;
    Fx->m[8][7] = -T*wqx;
    Fx->m[8][8] = 1;
    Fx->m[8][9] = -T*vqy;
    Fx->m[8][10] = T*vqx;
    Fx->m[8][11] = 0;

    Fx->m[9][0] = 0;
    Fx->m[9][1] = 0;
    Fx->m[9][2] = 0;
    Fx->m[9][3] = 0;
    Fx->m[9][4] = 0;
    Fx->m[9][5] = 0;
    Fx->m[9][6] = 0;
    Fx->m[9][7] = 0;
    Fx->m[9][8] = 0;
    Fx->m[9][9] = 1;
    Fx->m[9][10] = (T*(wqz*(IYY - IZZ) + IZZM*(w[0] - w[1] + w[2] - w[3])))/IXX;
    Fx->m[9][11] = (T*wqy*(IYY - IZZ))/IXX;

    Fx->m[10][0] = 0;
    Fx->m[10][1] = 0;
    Fx->m[10][2] = 0;
    Fx->m[10][3] = 0;
    Fx->m[10][4] = 0;
    Fx->m[10][5] = 0;
    Fx->m[10][6] = 0;
    Fx->m[10][7] = 0;
    Fx->m[10][8] = 0;
    Fx->m[10][9] = -(T*(wqz*(IXX - IZZ) - IZZM*(w[0]-w[1]+w[2]-w[3])))/IYY;
    Fx->m[10][10] = 1;
    Fx->m[10][11] = -(T*wqx*(IXX - IZZ))/IYY;

    Fx->m[11][0] = 0;
    Fx->m[11][1] = 0;
    Fx->m[11][2] = 0;
    Fx->m[11][3] = 0;
    Fx->m[11][4] = 0;
    Fx->m[11][5] = 0;
    Fx->m[11][6] = 0;
    Fx->m[11][7] = 0;
    Fx->m[11][8] = 0;
    Fx->m[11][9] = 0;
    Fx->m[11][10] = 0;
    Fx->m[11][11] = 1;
}

int H_init()
{
    int retval;
    H = uquad_mat_alloc(10,12);
    retval = uquad_mat_zeros(H);
    err_propagate(retval);
    H->m[0][3]=1;
    H->m[1][4]=1;
    H->m[2][5]=1;
    H->m[6][9]=1;
    H->m[7][10]=1;
    H->m[8][11]=1;
    H->m[9][2]=1;
}

kalman_io_t* kalman_init()
{
    kalman_io_t* kalman_io_data = (kalman_io_t*)malloc(sizeof(kalman_io_t));
    kalman_io_data->x_hat = uquad_mat_alloc(12,1);    
    kalman_io_data->x_ = uquad_mat_alloc(12,1);
    kalman_io_data->u = uquad_mat_alloc(4,1);
    kalman_io_data->z = uquad_mat_alloc(10,1);
    kalman_io_data->Q = uquad_mat_alloc(12,12);
    kalman_io_data->R = uquad_mat_alloc(10,10);
    kalman_io_data->P = uquad_mat_alloc(12,12);
    return kalman_io_data;
}

int store_data(kalman_io_t* k_io_data, uquad_mat_t* w, imu_data_t* data)
{
    k_io_data->u->m_full[0] = w->m_full[0];
    k_io_data->u->m_full[1] = w->m_full[1];
    k_io_data->u->m_full[2] = w->m_full[2];
    k_io_data->u->m_full[3] = w->m_full[3];

    k_io_data->z->m_full[0] = data->magn->m_full[1];
    k_io_data->z->m_full[1] = data->magn->m_full[2];
    k_io_data->z->m_full[2] = data->magn->m_full[3];
    k_io_data->z->m_full[3] = data->acc->m_full[1];
    k_io_data->z->m_full[4] = data->acc->m_full[2];
    k_io_data->z->m_full[5] = data->acc->m_full[3];
    k_io_data->z->m_full[6] = data->gyro->m_full[1];
    k_io_data->z->m_full[7] = data->gyro->m_full[2];
    k_io_data->z->m_full[8] = data->gyro->m_full[3];
    k_io_data->z->m_full[9] = data->alt;

    k_io_data->T = data->T_us/1000000;
}
