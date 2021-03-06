% -------------------------------------------------------------------------
% Modelo fisico
% psi   = wqx+wqz*tan(fi)*cos(psi)+wqy*tan(fi)*sin(psi);
% phi   = wqy*cos(psi)-wqz*sin(psi);
% theta = wqz*cos(psi)/cos(fi)+wqy*sin(psi)/cos(fi);
% wqx   = ( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w1-w2+w3-w4)+L*(T2-T4) )/Ixx;
% wqy   = ( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w1-w2+w3-w4)+L*(T3-T1) )/Iyy;
% wqz   = ( -Izzm*(dw1-dw2+dw3-dw4)+Q1-Q2+Q3-Q4 )/Izz;
% 
% x = [psi phi tehta wqx wqy wqz ]
% 
% A partir de los datos del gyro se hace un Kalman para la estimación de la
% velocidad angular en las 3 direcciones. A partir de los datos del
% magnetometro, convertidos a angulos de euler en mong_conv se estiman los
% angulos con el mismo filtro de kalman.
% -------------------------------------------------------------------------

close all
clear all
clc

%% Observaciones y constantes

% [acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('/gyro/logs/zv3y00',0);
% [acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('tests/mongoose/magnetometro/data_horizontal/x00z00',0);
% [a,w,euler] = mong_conv(acrud,wcrud,mcrud,0);
[acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('log_kalman_zparriba',0);
[a,w,euler] = mong_conv(acrud,wcrud/14.375,mcrud,0);

fs = 30;                % Frecuencia en Hz
T  = 1/fs;              % Periodo de muestreo en segundos
N  = size(a,1);         % Cantidad de muestras de las observaciones
Ns = 6;                 % N states: cantidad de variables de estado
g  = 9.81;
z  = [euler w];         % Observaciones
% z  = z+0*randn(N,Ns);   % Agregado de ruido

%% Constantes

Ixx  = 5;               % Tensor de inercia del quad - según x
Iyy  = 5;               % Tensor de inercia del quad - según y
Izz  = 10;              % Tensor de inercia del quad - según z
Izzm = 2;               % Tensor de inercia de los motores - segun z
L    = 0.3;             % Largo en metros del los brazos del quad

%% Entradas

w  = 10*ones(N,4);                  % Velocidades angulares de los motores en rad/s. Cada columna corresponde con 1 motor
dw = zeros(N,4);                    % Derivada de w. Cada columna corresponde a 1 motor
TM = 3.5296e-5*w.^2-4.9293e-4.*w;   % Fuerzas ejercidas por los motores en N. Cada columna corresponde a 1 motor.
D  = 3.4734e-6*w.^2-1.3205e-4.*w;   % Torque de Drag ejercido por los motores en N*m. Cada columna corresponde a cada motor

%% Kalman

sigma_a = load('acc','sigma');sigma_a=sigma_a.sigma;
sigma_w = load('gyro','sigma');sigma_w=sigma_w.sigma;
sigma_m = load('mag','sigma');sigma_m=sigma_m.sigma;

% Q = sigma_w^2*eye(Ns);  % 10*eye(Ns)
Q = 100000000000*eye(Ns);
R = diag(1000000000000*[100 100 100 100 100 100]);        % 1000*eye(Ns)

P = 1*eye(Ns);
x_est=zeros(N,Ns);
x_est(1,:)=z(1,:);

f = @(psi,phi,theta,wqx,wqy,wqz,w,dw,TM,D) [ ...    
    psi   + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
    phi   + T*( wqy*cos(psi)-wqz*sin(psi));
    theta + T*( wqz*cos(psi)/cos(phi)+wqy*sin(psi)/cos(phi));
    wqx   + T*( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TM(2)-TM(4)) )/Ixx ;
    wqy   + T*( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TM(3)-TM(1)) )/Iyy;
    wqz   + T*( -Izzm*(dw(1)-dw(2)+dw(3)-dw(4))+D(1)-D(2)+D(3)-D(4) )/Izz ...
    ];

h = @(psi,phi,theta,wqx,wqy,wqz) [ ...
    psi ; 
    phi ; 
    theta ; 
    wqx ; 
    wqy ; 
    wqz ...
    ];    

F = @(psi,phi,theta,wqx,wqy,wqz,w) ...
    [...
    T*(wqy*cos(psi)*tan(phi) - wqz*sin(psi)*tan(phi)) + 1,           T*(wqz*cos(psi)*(tan(phi)^2 + 1) + wqy*sin(psi)*(tan(phi)^2 + 1)), 0,                                                       T,                                    T*sin(psi)*tan(phi),      T*cos(psi)*tan(phi);
                         -T*(wqz*cos(psi) + wqy*sin(psi)),                                                                           1, 0,                                                       0,                                             T*cos(psi),              -T*sin(psi);
    T*((wqy*cos(psi))/cos(phi) - (wqz*sin(psi))/cos(phi)), T*((wqz*cos(psi)*sin(phi))/cos(phi)^2 + (wqy*sin(phi)*sin(psi))/cos(phi)^2), 1,                                                       0,                                  (T*sin(psi))/cos(phi),    (T*cos(psi))/cos(phi);
                                                        0,                                                                           0, 0,                                                       1, (T*(wqz*(Iyy - Izz) + Izzm*(w(1)-w(2)+w(3)-w(4))))/Ixx,  (T*wqy*(Iyy - Izz))/Ixx;
                                                        0,                                                                           0, 0, -(T*(wqz*(Ixx - Izz) - Izzm*(w(1)-w(2)+w(3)-w(4))))/Iyy,                                                      1, -(T*wqx*(Ixx - Izz))/Iyy;
                                                        0,                                                                           0, 0,                                                       0,                                                      0,                        1 ...
    ];

H = @() ...
    [ ...
    1, 0, 0, 0, 0, 0 ;
    0, 1, 0, 0, 0, 0 ;
    0, 0, 1, 0, 0, 0 ;
    0, 0, 0, 1, 0, 0 ;
    0, 0, 0, 0, 1, 0 ;
    0, 0, 0, 0, 0, 1 ...
    ];

for i=2:N
    % Prediction
    x_ = f(x_est(i-1,1),x_est(i-1,2),x_est(i-1,3),x_est(i-1,4),x_est(i-1,5),x_est(i-1,6),w(i-1,:),dw(i-1,:),TM(i-1,:),D(i-1,:));
    Fk_1 = F(x_est(i-1,1),x_est(i-1,2),x_est(i-1,3),x_est(i-1,4),x_est(i-1,5),x_est(i-1,6),w(i-1,:));
    P_ = Fk_1 * P * Fk_1'+ Q; 
    
    % Update
    yk         = z(i,:)' - h(x_(1),x_(2),x_(3),x_(4),x_(5),x_(6));
    Hk         = H();
    Sk         = Hk*P_*Hk' + R;
    Kk         = P_*Hk'*Sk^-1;
    x_est(i,:) = x_ + Kk*yk;
    P          = (eye(Ns)-Kk*Hk)*P_;
end


%% Plots

figure()
    plot(z(:,1),'b--')
    hold on; grid
    plot(z(:,2),'r--')
    plot(z(:,3),'g--')    
    plot([x_est(1:end,1)],'b')
    plot([x_est(1:end,2)],'r')
    plot([x_est(1:end,3)],'g')
    legend('\psi','\phi','\theta','\psi','\phi','\theta')
    hold off    
    
figure()
    plot(z(:,4),'b--')
    hold on; grid
    plot(z(:,5),'r--')
    plot(z(:,6),'g--')    
    plot([x_est(1:end,4)],'b')
    plot([x_est(1:end,5)],'r')
    plot([x_est(1:end,6)],'g')
    legend('w_x','w_y','w_z','w_x','w_y','w_z')
    hold off

figure()
subplot(211)
    plot(z(:,1)-[x_est(1:end,1)],'b')
    hold on; grid
    plot(z(:,2)-[x_est(1:end,2)],'r')
    plot(z(:,3)-[x_est(1:end,3)],'g')    
    legend('\psi','\phi','\theta')
    title('Errores en angulos')
    hold off
subplot(212)
    plot(z(:,4)-[x_est(1:end,4)],'b')
    hold on; grid
    plot(z(:,5)-[x_est(1:end,5)],'r')
    plot(z(:,6)-[x_est(1:end,6)],'g')    
    legend('w_x','w_y','w_z')
    title('Errores en velocidades angulares')
    hold off