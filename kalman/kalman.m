% -------------------------------------------------------------------------
% Modelo fisico
% wqx   = ( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w1-w2+w3-w4)+L*(T2-T4) )/Ixx;
% wqy   = ( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w1-w2+w3-w4)+L*(T3-T1) )/Iyy;
% wqz   = ( -Izzm*(dw1-dw2+dw3-dw4)+Q1-Q2+Q3-Q4 )/Izz;
% theta = wqz*cos(psi)/cos(fi)+wqy*sin(psi)/cos(fi);
% fi    = wqy*cos(psi)-wqz*sin(psi);
% psi   = wqx+wqz*tan(fi)*cos(psi)+wqy*tan(fi)*sin(psi);
% 
% x = [wqx wqy wqz tehta fi psi]
% 
% A partir de los datos del gyro se hace un Kalman para la estimación de la
% velocidad angular en las 3 direcciones. Proximamente se integrará con la
% medida del magnetometro
% -------------------------------------------------------------------------

close all
clear all
clc

%% Datos ejemplo - observaciones

[acrud,wcrud,ccrud,tcrud,bcrud]=mong_read('/gyro/logs/zv3y00',0);
[a,w] = mong_conv(acrud,wcrud,ccrud,0);

N = size(a,1);
z = w+0*randn(N,3);

%% Constantes

Ixx  = 5;
Iyy  = 5;
Izz  = 10;
Izzm = 2;
L    = 0.3;
Q1   = 0.1*ones(N,1);
Q2   = 0.1*ones(N,1);
Q3   = 0.1*ones(N,1);
Q4   = 0.1*ones(N,1);

%% Entradas

w1  = 10*ones(N,1);
w2  = 10*ones(N,1);
w3  = 10*ones(N,1);
w4  = 10*ones(N,1);
dw1 = zeros(N,1);
dw2 = zeros(N,1);
dw3 = zeros(N,1);
dw4 = zeros(N,1);

T1 = 3.5296e-5*w1.^2-4.9293e-4.*w1;
T2 = 3.5296e-5*w2.^2-4.9293e-4.*w2;
T3 = 3.5296e-5*w3.^2-4.9293e-4.*w3;
T4 = 3.5296e-5*w4.^2-4.9293e-4.*w4;

%% Kalman

sigma_a = load('acc','sigma');sigma_a=sigma_a.sigma;
sigma_g = load('gyro','sigma');sigma_g=sigma_g.sigma;
n1      = sigma_a*ones(N,1);
n2      = sigma_g*ones(N,1);

% R       = zeros(3,3);%sigma_a^2*eye(3);         % TODO OJO DUDOSO

Q       = 10*eye(3);         % TODO OJO DUDOSO
% R       = sigma_g^2*eye(3);
R       = 10000*eye(3);

% Q=cov(sigma_g*randn(N,3));
% R=Q/20;

P = 1*eye(3);
x_est=zeros(N,3);
x_est(1,:)=z(1,:);

K=zeros(3,3,3);
for i=2:N
    % Prediction
    x_ = [ ...
        x_est(i-1,1) + ( x_est(i-1,2)*x_est(i-1,3)*(Iyy-Izz)+x_est(i-1,2)*Izzm*(w1(i-1)-w2(i-1)+w3(i-1)-w4(i-1))+L*(T2(i-1)-T4(i-1)) )/Ixx ; ...
        x_est(i-1,2) + ( x_est(i-1,1)*x_est(i-1,3)*(Izz-Ixx)+x_est(i-1,1)*Izzm*(w1(i-1)-w2(i-1)+w3(i-1)-w4(i-1))+L*(T3(i-1)-T1(i-1)) )/Iyy ; ...
        x_est(i-1,3) + (-Izzm*(dw1(i-1)-dw2(i-1)+dw3(i-1)-dw4(i-1))+Q1(i-1)-Q2(i-1)+Q3(i-1)-Q4(i-1) )/Izz ...
        ];    
    F  = [ 0 (x_est(i-1,3)*(Iyy-Izz)+Izzm*(w1(i-1)-w2(i-1)+w3(i-1)-w4(i-1)) )/Ixx ( x_est(i-1,2)*(Iyy-Izz))/Ixx ; ...
           (x_est(i-1,3)*(Izz-Ixx)+Izzm*(w1(i-1)-w2(i-1)+w3(i-1)-w4(i-1)) )/Iyy 0 ( x_est(i-1,1)*(Izz-Ixx))/Iyy ;
           0 0 0 ];       
    P_ = F * P * F'+ Q; 
    
    % Update
    y          = z(i,:)' - x_ ;
    s          = P_ + R; 
    K          = P_*s^-1;
    x_est(i,:) = x_ + K*y;
    P          = (eye(3)-K)*P_;
    K(:,:,i)=K;
end

figure()
    plot(z(:,1),'k')
    hold on
    plot(z(:,2),'k')
    plot(z(:,3),'k')    
    plot([0;x_est(2:end,1)])
    plot([0;x_est(2:end,2)],'r')
    plot([0;x_est(2:end,3)],'g')
    legend('w_x','w_y','w_z','w_x','w_y','w_z')
%     legend('posta','kalman')
    hold off
