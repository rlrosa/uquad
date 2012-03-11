close all
clear all
clc

%% Observaciones y constantes

% [acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('/gyro/logs/zv3y00',0);
% [acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('tests/mongoose/magnetometro/data_horizontal/z00y00',0);
[acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('log_kalman_zparriba',0);
[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0);

fs = 30;                % Frecuencia en Hz
T  = 1/fs;              % Periodo de muestreo en segundos
N  = size(a,1);         % Cantidad de muestras de las observaciones
Ns = 6;                % N states: cantidad de variables de estado
% z  = [euler a w];       % Observaciones
z = a;
% z  = [zeros(N,1) -90*ones(N,1) -72.5*ones(N,1) a zeros(N,3)];       % Observaciones
% z  = z+0*randn(N,Ns); % Agregado de ruido
% euler = [zeros(N,1) -90*ones(N,1) -72.5*ones(N,1)];
% euler = [0*ones(N,1) 0*ones(N,1) 0*ones(N,1)];
% euler = [180*ones(N,1) 0*ones(N,1) 168.61*ones(N,1)];
% euler = [0*ones(N,1) -90*ones(N,1) -100*ones(N,1)];
euler = [0*ones(N,1) 0*ones(N,1) 172.4*ones(N,1)];
ww = zeros(N,3);

%% Constantes

Ixx  = 2.32e-2;         % Tensor de inercia del quad - según x
Iyy  = 2.32e-2;         % Tensor de inercia del quad - según y
Izz  = 4.37e-2;         % Tensor de inercia del quad - según z
Izzm = 1.54e-5;         % Tensor de inercia de los motores - segun z
L    = 0.29;            % Largo en metros del los brazos del quad
M    = 1.541;           % Masa del Quad en kg
g    = 9.81;            % Aceleracion gravitatoria

sigma_a = load('acc','sigma');sigma_a=sigma_a.sigma;
sigma_w = load('gyro','sigma');sigma_w=sigma_w.sigma;
sigma_m = load('mag','sigma');sigma_m=sigma_m.sigma;


%% Entradas

w  = 0*ones(N,4);                  % Velocidades angulares de los motores en rad/s. Cada columna corresponde con 1 motor
dw = zeros(N,4);                   % Derivada de w. Cada columna corresponde a 1 motor
TM = 3.5296e-5*w.^2-4.9293e-4.*w;  % Fuerzas ejercidas por los motores en N. Cada columna corresponde a 1 motor.
D  = 3.4734e-6*w.^2-1.3205e-4.*w;  % Torque de Drag ejercido por los motores en N*m. Cada columna corresponde a cada motor

%% Kalman | vector de estados = [psi phi theta vqx vqy vqz wqx wqy wqz dvqx dvqy dvqz]

% f = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM) [ ...    
%     vqx   + T*( vqy*wqz-vqz*wqy+g*sin(phi));
%     vqy   + T*( vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi));
%     vqz   + T*( vqx*wqy-vqy*wqx-g*cos(phi)*cos(psi)+1/M*(TM(1)+TM(2)+TM(3)+TM(4)));
%     0;
%     0;
%     1/M*(TM(1)+TM(2)+TM(3)+TM(4))
%     ];


f = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM) [ ...    
    vqx   + T*( vqy*wqz-vqz*wqy+g*sin(phi));
    vqy   + T*( vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi));
    vqz   + T*( vqx*wqy-vqy*wqx-g*cos(phi)*cos(psi)+1/M*(TM(1)+TM(2)+TM(3)+TM(4)));
    vqy*wqz-vqz*wqy+g*sin(phi);
    vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi);
    vqx*wqy-vqy*wqz-g*cos(phi)*cos(psi)+1/M*(TM(1)+TM(2)+TM(3)+TM(4)) ...
    ];


h = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,dvqx,dvqy,dvqz) [ ...% Devuelve [psi;phi;theta;ax;ay;az;wqx;wqy;wqz] --> misma forma que z. Sale tmb del el imu_conv
    dvqx-vqy*wqz+vqz*wqy+g*sin(phi);
    dvqy-vqz*wqx+vqx*wqz-g*cos(phi)*sin(psi);
    dvqz-vqx*wqy+vqy*wqx-g*cos(phi)*cos(psi);
    ];    

% F = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,w,dw,TM,D) [ ... 
%       1,  T*wqz, -T*wqy, 0, 0, 0
%  -T*wqz,      1,  T*wqx, 0, 0, 0
%   T*wqy, -T*wqx,      1, 0, 0, 0
%       0,      0,      0, 0, 0, 0
%       0,      0,      0, 0, 0, 0
%       0,      0,      0, 0, 0, 0];

F = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM) [ ... 
      1,  T*wqz, -T*wqy, 0, 0, 0
 -T*wqz,      1,  T*wqx, 0, 0, 0
  T*wqy, -T*wqx,      1, 0, 0, 0
      0,    wqz,   -wqy, 0, 0, 0
   -wqz,      0,    wqx, 0, 0, 0
    wqy,   -wqz,      0, 0, 0, 0];
 
  
  
  
H =@(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz) [ ... 
    0, -wqz,  wqy, 1, 0, 0
  wqz,    0, -wqx, 0, 1, 0
 -wqy,  wqx,    0, 0, 0, 1];

% Q = sigma_w^2*eye(Ns);  % 10*eye(Ns)
Q = diag(1/1*sigma_w^2*[1 1 1 1 1 1]);
R = diag(1*[100 100 100]);        % 1000*eye(Ns)

P = 1*eye(Ns);
x_hat=zeros(N,Ns);


for i=2:N
    % Prediction    
    x_   = f(euler(i-1,1),euler(i-1,2),euler(i-1,3),x_hat(i-1,1),x_hat(i-1,2),x_hat(i-1,3),ww(i-1,1),ww(i-1,2),ww(i-1,3),TM(i-1,:));
    Fk_1 = F(euler(i-1,1),euler(i-1,2),euler(i-1,3),x_hat(i-1,1),x_hat(i-1,2),x_hat(i-1,3),ww(i-1,1),ww(i-1,2),ww(i-1,3),TM(i-1,:));
    P_ = Fk_1 * P * Fk_1'+ Q; 
    
    % Update              
    yk         = z(i,:)' - h(euler(i-1,1),euler(i-1,2),euler(i-1,3),x_(1),x_(2),x_(3),ww(i-1,1),ww(i-1,2),ww(i-1,3),x_(4),x_(5),x_(6));
                % (psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz)
    Hk         = H(euler(i-1,1),euler(i-1,2),euler(i-1,3),x_(1),x_(2),x_(3),ww(i-1,1),ww(i-1,2),ww(i-1,3));
    Sk         = Hk*P_*Hk' + R;
    Kk         = P_*Hk'*Sk^-1;
    x_hat(i,:) = x_ + Kk*yk;
    P          = (eye(Ns)-Kk*Hk)*P_;
end


% Plots

% figure()

% subplot(211)
%     plot(z(:,1),'b--')
%     hold on; grid
%     plot(z(:,2),'r--')
%     plot(z(:,3),'g--')    
%     plot([x_hat(1:end,1)],'b')
%     plot([x_hat(1:end,2)],'r')
%     plot([x_hat(1:end,3)],'g')
%     legend('\psi','\phi','\theta','\psi','\phi','\theta')
%     hold off
%     
% subplot(212)
%     plot([x_hat(1:end,4)],'b')
%     hold on; grid
%     plot([x_hat(1:end,5)],'r')
%     plot([x_hat(1:end,6)],'g')
%     legend('vqx','vqy','vqz')
%     hold off
    
subplot(211)
    plot([x_hat(1:end,1)],'b')
    hold on; grid
    plot([x_hat(1:end,2)],'r')
    plot([x_hat(1:end,3)],'g')
    legend('vqx','vqy','vqz')
    hold off
    
subplot(212)
    plot([x_hat(1:end,4)],'b')
    hold on; grid    
    plot([x_hat(1:end,5)],'r')
    plot([x_hat(1:end,6)],'g')
    legend('dvqx','dvqy','dvqz')
    hold off