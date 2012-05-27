% function [kin, imu_raw, imu_data, x_hat_c, wlog, z, int, imu_avg] = plot_c(path)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
% function [kin, imu_data, x_hat_c, wlog, z] = plot_c(path)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

path ='tests/main/logs/2012_05_25_1_05_mejoro_altura/';

if(~exist('path','var'))
  path = 'src/build/main/';
  fprintf('Using default path: %s\n',path);
end

z = [];

kin      = load([path 'kalman_in.log']);
x_hat_c  = load([path 'x_hat.log'    ]);
wlog     = load([path 'w.log'        ]);
imu_raw  = load_if_exist([path 'imu_raw.log'  ]);
imu_data = load_if_exist([path 'imu_data.log' ]);
imu_avg  = load_if_exist([path 'imu_avg.log' ]);
int      = load_if_exist([path 'int.log'      ]);

lens     = [length(kin)      ...
            length(x_hat_c)  ...
            length(wlog)];
if(sum(lens ~= lens(1)) ~= 0)
  len_min = min(lens);
  fprintf('WARN: Will trim by:\n\tkin:\t\t%d\n\tx_hat:\t\t%d\n\tw:\t\t%d\n\t\n', ...
    length(kin)      - len_min, ...
    length(x_hat_c)  - len_min, ...
    length(wlog)     - len_min);    
  kin      = kin     (1:len_min,:);
  x_hat_c  = x_hat_c (1:len_min,:);
  wlog     = wlog    (1:len_min,:);
end

z        = kin2z([ones(length(kin),1) kin]);

if(mean(kin(:,1)) < .1)
  % logs con delta tiempos
  kin(1,1) = 0; % bug, era enorme.
  t = cumsum(kin(:,1));
else
  % logs con t desde ./main
  t = kin(:,1);
end

%% Plots

% timestamp kin coincide con x_hat_c, por eso x_hat_c no tiene.
plot_main(x_hat_c,t,z,t);
plot_w(wlog)

figure
    plot(int(:,1),int(:,2),'color',blue1,'linewidth',2.5);
    hold on
    plot(int(:,1),int(:,3),'color',green1,'linewidth',2.5);
    plot(int(:,1),int(:,5),'color',orange1,'linewidth',2.5);
    plot(int(:,1),int(:,4),'color',red2,'linewidth',2.5);
    legend('\fontsize{13}\psi [rad*s]','\fontsize{13}\phi [rad*s]','\fontsize{13}\theta [rad*s]','\fontsize{13}z [m*s]')
    xlabel('\fontsize{15}Tiempo [s]')
    ylabel('\fontsize{15}Integral del error respecto al set point')

% figure; 
%     plot(wlog(:,2)+wlog(:,4)-wlog(:,3)-wlog(:,5),'r','linewidth',3); 
%     title('diferencia entre velocidades angulares (adelante+atras)-(derecha+izquierda)'); 
%     legend('Giro en z')
