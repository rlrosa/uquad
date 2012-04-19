% function [kin, imu_data, x_hat_c, wlog, kin_cut, z] = plot_c(path)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
% function [kin, imu_data, x_hat_c, wlog, kin_cut, z] = plot_c(path)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
%  path='2012_04_16_1_1_arranco_loquito/';
%  path='2012_04_16_1_2_subio_derecho_8_states_no_int/';
  %path='2012_04_16_1_3_aguanto_bastante/';
 path='2012_04_16_1_4_8_states_con_integral/';
%  path='2012_04_16_1_5_igual_q_el_anterior/';
 %path='2012_04_16_1_6_sp_z_1m_sin_integrador/';
% path='2012_04_16_1_7_igual_q_anterior/';
 % path='2012_04_16_1_8_igual_anterior/';

path =['tests/main/logs/' path];
if(~exist('path','var'))
  path = 'src/build/main/';
  fprintf('Using default path: %s',path);
end

kin      = load([path 'kalman_in.log']);
imu_data = load([path 'imu_data.log']);
x_hat_c  = load([path 'x_hat.log'    ]);
wlog     = load([path 'w.log'        ]);

lens     = [length(kin)      ...
            length(imu_data) ...
            length(x_hat_c)  ...
            length(wlog)];
if(sum(diff(lens)) ~= 0)
  len_min = min(lens);
  fprintf('WARN: Will trim by:\n\tkin:\t\t%d\n\timu_data:\t%d\n\tx_hat:\t\t%d\n\tw:\t\t%d\n\t\n', ...
    length(kin)      - len_min, ...
    length(imu_data) - len_min, ...
    length(x_hat_c)  - len_min, ...
    length(wlog)     - len_min);    
  kin      = kin     (1:len_min,:);
  imu_data = imu_data(len_min:end-len_min+1,:);
  x_hat_c  = x_hat_c (1:len_min,:);
  wlog     = wlog    (1:len_min,:);
end

kin_cut  = kin(end-length(x_hat_c)+1:end,:);

z        = kin2z([ones(length(kin_cut),1) kin_cut]);

if(mean(kin(:,1)) < .1)
  % logs con delta tiempos
  kin(1,1) = 0; % bug, era enorme.
  t = cumsum(kin(:,1));
else
  % logs con t desde ./main
  t = kin(:,1);
end
t_z     = t(length(t) - length(z) + 1:end);
t_x_hat = t(length(t) - length(x_hat_c) + 1:end);

% timestamp kin coincide con x_hat_c, por eso x_hat_c no tiene.
plot_main(x_hat_c,t_x_hat, ...
  z,t_z);

plot_w(wlog)

% figure; 
%     plot(wlog(:,2)+wlog(:,4)-wlog(:,3)-wlog(:,5),'r','linewidth',3); 
%     title('diferencia entre velocidades angulares (adelante+atras)-(derecha+izquierda)'); 
%     legend('Giro en z')
