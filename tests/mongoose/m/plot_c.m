function [kin, imu_data, x_hat_c, wlog, kin_cut, z] = plot_c(path)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
% function [kin, imu_data, x_hat_c, wlog, kin_cut, z] = plot_c(path)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

kin      = load([path 'kalman_in.log']);
imu_data = load([path 'imu_data.log']);
x_hat_c  = load([path 'x_hat.log'    ]);
wlog     = load([path 'w.log'        ]);
kin_cut  = kin(end-length(x_hat_c)+1:end,:);

z        = kin2z([ones(length(kin_cut),1) kin_cut]);

% timestamp kin coincide con x_hat_c, por eso x_hat_c no tiene.
plot_main(x_hat_c,kin(:,1), ...
  z,kin_cut(:,1));

plot_w(wlog)

figure; 
    plot(-wlog(:,2)-wlog(:,4)+wlog(:,3)+wlog(:,5),'r','linewidth',3); 
    title('diferencia entre velocidades angulares (adelante+atras)-(derecha+izquierda)'); 
    legend('Giro en z')
