function plot_c(path)

% kin     = load([path 'kalman_in.log']);
kin     = load([path 'imu_data.log']);
x_hat_c = load([path 'x_hat.log'    ]);
wlog    = load([path 'w.log'        ]);
T       = load([path 'kalman_in.log'    ]); T = 1e-6*T(end-length(x_hat_c)+1:end,4);
% z = kin2z(kin);
kin_cut = kin(end-length(x_hat_c)+1:end,:);
z = kin2z([ones(length(kin_cut),1) kin_cut]);

plot_main(x_hat_c,z,T)
plot_w([zeros(1,4);wlog(:,2:end)],T)

figure; 
    plot(-wlog(:,2)-wlog(:,4)+wlog(:,3)+wlog(:,5),'r','linewidth',3); 
    title('diferencia entre velocidades angulares (adelante+atras)-(derecha+izquierda)'); 
    legend('Giro en z')
