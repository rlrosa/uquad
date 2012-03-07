close all
clear all
clc

%% Observaciones y constantes

% [acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('/gyro/logs/zv3y00',0);
[acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('tests/mongoose/magnetometro/data_horizontal/z00y30',0);
[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0);

fs = 30;                % Frecuencia en Hz
T  = 1/fs;              % Periodo de muestreo en segundos
N  = size(a,1);         % Cantidad de muestras de las observaciones
Ns = 9;                 % N states: cantidad de variables de estado
g  = 9.81;
z  = [euler a w];             % Observaciones
% z  = z+0*randn(N,Ns); % Agregado de ruido

%% Constantes

Ixx  = 5;               % Tensor de inercia del quad - según x
Iyy  = 5;               % Tensor de inercia del quad - según y
Izz  = 10;              % Tensor de inercia del quad - según z
Izzm = 2;               % Tensor de inercia de los motores - segun z
L    = 0.3;             % Largo en metros del los brazos del quad
M    = 1.3;             % Masa del Quad en kg
g    = 9.81;            % Aceleracion gravitatoria

sigma_a = load('acc','sigma');sigma_a=sigma_a.sigma;
sigma_w = load('gyro','sigma');sigma_w=sigma_w.sigma;
sigma_m = load('mag','sigma');sigma_m=sigma_m.sigma;


%% Entradas

w  = 10*ones(N,4);                  % Velocidades angulares de los motores en rad/s. Cada columna corresponde con 1 motor
dw = zeros(N,4);                    % Derivada de w. Cada columna corresponde a 1 motor
TM = 3.5296e-5*w.^2-4.9293e-4.*w;   % Fuerzas ejercidas por los motores en N. Cada columna corresponde a 1 motor.
D  = 3.4734e-6*w.^2-1.3205e-4.*w;   % Torque de Drag ejercido por los motores en N*m. Cada columna corresponde a cada motor

%% Kalman | vector de estados = [psi phi theta vqx vqy vqz wqx wqy wqz dvqx dvqy dvqz]

f = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,w,dw,TM,D) [ ...    
    psi   + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
    phi   + T*( wqy*cos(psi)-wqz*sin(psi));
    theta + T*( wqz*cos(psi)/cos(phi)+wqy*sin(psi)/cos(phi));
    vqx   + T*( vqy*wqz-vqz*wqy+g*sin(phi));
    vqy   + T*( vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi));
    vqz   + T*( vqx*wqy-vqy*wqx-g*cos(phi)*cos(psi)+1/M*(TM(1)+TM(2)+TM(3)+TM(4)));
    wqx   + T*( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TM(2)-TM(4)) )/Ixx ;
    wqy   + T*( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TM(3)-TM(1)) )/Iyy;
    wqz   + T*( -Izzm*(dw(1)-dw(2)+dw(3)-dw(4))+D(1)-D(2)+D(3)-D(4) )/Izz;
    ...
    ];

h = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM) [ ...% Devuelve [psi;phi;theta;ax;ay;az;wqx;wqy;wqz] --> misma forma que z. Sale tmb del el imu_conv
    psi ; 
    phi ; 
    theta ; 
    vqy*wqz-vqz*wqy;
    vqz*wqx-vqx*wqz;
    vqx*wqy-vqy*wqx+1/M*(TM(1)+TM(2)+TM(3)+TM(4));
    wqx ; 
    wqy ; 
    wqz ...
    ];    

F = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,w,dw,TM,D) [ ... 
 T*(wqy*cos(psi)*tan(phi) - wqz*sin(psi)*tan(phi)) + 1,           T*(wqz*cos(psi)*(tan(phi)^2 + 1) + wqy*sin(psi)*(tan(phi)^2 + 1)), 0,      0,      0,      0,                                                     T,                                  T*sin(psi)*tan(phi),      T*cos(psi)*tan(phi);
                      -T*(wqz*cos(psi) + wqy*sin(psi)),                                                                           1, 0,      0,      0,      0,                                                     0,                                           T*cos(psi),              -T*sin(psi);
 T*((wqy*cos(psi))/cos(phi) - (wqz*sin(psi))/cos(phi)), T*((wqz*cos(psi)*sin(phi))/cos(phi)^2 + (wqy*sin(phi)*sin(psi))/cos(phi)^2), 1,      0,      0,      0,                                                     0,                                (T*sin(psi))/cos(phi),    (T*cos(psi))/cos(phi);
                                                     0,                                                                T*g*cos(phi), 0,      1,  T*wqz, -T*wqy,                                                     0,                                               -T*vqz,                    T*vqy;
                                -T*g*cos(phi)*cos(psi),                                                       T*g*sin(phi)*sin(psi), 0, -T*wqz,      1,  T*wqx,                                                 T*vqz,                                                    0,                   -T*vqx;
                                 T*g*cos(phi)*sin(psi),                                                       T*g*cos(psi)*sin(phi), 0,  T*wqy, -T*wqx,      1,                                                -T*vqy,                                                T*vqx,                        0;
                                                     0,                                                                           0, 0,      0,      0,      0,                                                     1, (T*(wqz*(Iyy - Izz) + Izzm*(w(1) - w(2) + w(3) - w(4))))/Ixx,  (T*wqy*(Iyy - Izz))/Ixx;
                                                     0,                                                                           0, 0,      0,      0,      0, -(T*(wqz*(Ixx - Izz) - Izzm*(w(1) - w(2) + w(3) - w(4))))/Iyy,                                                    1, -(T*wqx*(Ixx - Izz))/Iyy;
                                                     0,                                                                           0, 0,      0,      0,      0,                                                     0,                                                    0,                        1];
 
 
H = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz) [...  
 1, 0, 0,    0,    0,    0,    0,    0,    0;
 0, 1, 0,    0,    0,    0,    0,    0,    0;
 0, 0, 1,    0,    0,    0,    0,    0,    0;
 0, 0, 0,    0,  wqz, -wqy,    0, -vqz,  vqy;
 0, 0, 0, -wqz,    0,  wqx,  vqz,    0, -vqx;
 0, 0, 0,  wqy, -wqx,    0, -vqy,  vqx,    0;
 0, 0, 0,    0,    0,    0,    1,    0,    0;
 0, 0, 0,    0,    0,    0,    0,    1,    0;
 0, 0, 0,    0,    0,    0,    0,    0,    1];


Q = sigma_w^2*eye(Ns);  % 10*eye(Ns)
R = diag(1*[10 10 10 100 100 100 100 100 100]);        % 1000*eye(Ns)

P = 1*eye(Ns);
x_hat=zeros(N,Ns);
% x_hat(1,:)=z(1,:);
psi_init   = z(1,4);
phi_init   = z(1,5);
theta_init = z(1,6);
wqx_init   = z(1,7);
wqy_init   = z(1,8);
wqz_init   = z(1,9);
vqx_init   = -(TM(1,1)*wqx_init + TM(1,2)*wqx_init + TM(1,3)*wqx_init + TM(1,4)*wqx_init - M*z(1,4)*wqx_init - M*z(1,5)*wqy_init - M*z(1,6)*wqx_init + M*g*wqx_init*sin(phi_init) - M*g*wqx_init*cos(phi_init)*cos(psi_init) - M*g*wqy_init*cos(phi_init)*sin(psi_init))/(M*wqy_init*(wqx_init - wqz_init));
vqy_init   = -(TM(1,1)*wqz_init + TM(1,2)*wqz_init + TM(1,3)*wqz_init + TM(1,4)*wqz_init - M*z(1,4)*wqx_init - M*z(1,5)*wqy_init - M*z(1,6)*wqz_init + M*g*wqx_init*sin(phi_init) - M*g*wqz_init*cos(phi_init)*cos(psi_init) - M*g*wqy_init*cos(phi_init)*sin(psi_init))/(M*wqz_init*(wqx_init - wqz_init));
vqz_init   = -(TM(1,1)*wqz_init + TM(1,2)*wqz_init + TM(1,3)*wqz_init + TM(1,4)*wqz_init - M*z(1,4)*wqz_init - M*z(1,5)*wqy_init - M*z(1,6)*wqz_init + M*g*wqz_init*sin(phi_init) - M*g*wqz_init*cos(phi_init)*cos(psi_init) - M*g*wqy_init*cos(phi_init)*sin(psi_init))/(M*wqy_init*(wqx_init - wqz_init));
x_hat(1,:)=[ psi_init phi_init theta_init vqx_init vqy_init vqz_init wqy_init wqx_init wqz_init];
clear psi_init; clear phi_init; clear theta_init; clear wqx_init; clear wqy_init; clear wqz_init; clear vqx_init; clear vqy_init; clear vqz_init;


for i=2:N
    % Prediction    
    x_   = f(x_hat(i-1,1),x_hat(i-1,2),x_hat(i-1,3),x_hat(i-1,4),x_hat(i-1,5),x_hat(i-1,6),x_hat(i-1,7),x_hat(i-1,8),x_hat(i-1,9),w(i-1,:),dw(i-1,:),TM(i-1,:),D(i-1,:));
    Fk_1 = F(x_hat(i-1,1),x_hat(i-1,2),x_hat(i-1,3),x_hat(i-1,4),x_hat(i-1,5),x_hat(i-1,6),x_hat(i-1,7),x_hat(i-1,8),x_hat(i-1,9),w(i-1,:),dw(i-1,:),TM(i-1,:),D(i-1,:));
    P_ = Fk_1 * P * Fk_1'+ Q; 
    
    % Update
    yk         = z(i,:)' - h(x_(1),x_(2),x_(3),x_(4),x_(5),x_(6),x_(7),x_(8),x_(9),TM(i-1,:));
    Hk         = H(x_hat(i-1,1),x_hat(i-1,2),x_hat(i-1,3),x_hat(i-1,4),x_hat(i-1,5),x_hat(i-1,6),x_hat(i-1,7),x_hat(i-1,8),x_hat(i-1,9));
    Sk         = Hk*P_*Hk' + R;
    Kk         = P_*Hk'*Sk^-1;
    x_hat(i,:) = x_ + Kk*yk;
    P          = (eye(Ns)-Kk*Hk)*P_;
end


% Plots

% figure()

subplot(311)
    plot(z(:,1),'b--')
    hold on; grid
    plot(z(:,2),'r--')
    plot(z(:,3),'g--')    
    plot([x_hat(1:end,1)],'b')
    plot([x_hat(1:end,2)],'r')
    plot([x_hat(1:end,3)],'g')
    legend('\psi','\phi','\theta','\psi','\phi','\theta')
    hold off
    
subplot(312)
    plot([x_hat(1:end,4)],'b')
    hold on; grid
    plot([x_hat(1:end,5)],'r')
    plot([x_hat(1:end,6)],'g')
    legend('vqx','vqy','vqz')
    hold off
    
subplot(313)
    plot(z(:,7),'b--')
    hold on; grid
    plot(z(:,8),'r--')
    plot(z(:,9),'g--')    
    plot([x_hat(1:end,7)],'b')
    plot([x_hat(1:end,8)],'r')
    plot([x_hat(1:end,9)],'g')
    legend('wqx','wqy','wqz','wqx','wqy','wqz')
    hold off