% function plot_c(path)

% folder = '2012_04_06_1_1_izquierda';
% folder = '2012_04_06_1_2_no_despego';
% folder = '2012_04_06_1_3_bastante_rico';
% folder = '2012_04_06_1_4_se_fue_de_boca';
% folder = '2012_04_06_1_5_casi_mata_pater';
% folder = '2012_04_06_1_5_casi_me_mata';
% folder = '2012_04_06_1_6_divino';
% folder = '2012_04_06_1_7_1m_sp';
% folder = '2012_04_06_1_8_tironeo';
% folder = '2012_04_06_1_9';
% folder = '2012_04_06_1_10_al_sur_no_le_gusta';
folder = '2012_04_06_1_11_rico_w_hover_hasta_321';

path = ['tests/main/logs/' folder '/'];

% kin     = load([path 'kalman_in.log']);
data    = load([path 'imu_data.log']);
x_hat_c = load([path 'x_hat.log'    ]);
wlog    = load([path 'w.log'        ]);
T       = load([path 'kalman_in.log'    ]); T = 1e-6*T(end-length(x_hat_c)+1:end,4);
% z = kin2z(kin);
data_cut = data(end-length(x_hat_c)+1:end,:);
z = kin2z([ones(length(data_cut),1) data_cut]);

plot_main(x_hat_c,z,T)
plot_w(wlog(200:end,2:end),T(200:end))
% plot_w([zeros(1,4);wlog(:,2:end)],T)

figure; 
    plot(wlog(:,2)+wlog(:,4)-wlog(:,3)-wlog(:,5),'r','linewidth',3); 
    title('diferencia entre velocidades angulares (adelante+atras)-(derecha+izquierda)'); 
    legend('Giro en z')
