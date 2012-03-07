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
[acrud,wcrud,mcrud,tcrud,bcrud]=mong_read('tests/mongoose/magnetometro/data_horizontal/x00z00',0);
[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0);

fs = 30;                % Frecuencia en Hz
T  = 1/fs;              % Periodo de muestreo en segundos
N  = size(a,1);         % Cantidad de muestras de las observaciones
Ns = 12;                % N states: cantidad de variables de estado
z  = [euler a w];       % Observaciones
% z  = z+0*randn(N,Ns);   % Agregado de ruido

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

%% Kalman

f = @(x,y,z,psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,ax,ay,az,w,dw,TM,D) [ ...    
    x     + T *(vqx*cos(phi)*cos(theta)+vqy*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vqz*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi)) ) ;
    y     + T *(vqx*cos(phi)*sin(theta)+vqy*(sin(theta)*sin(phi)*sin(psi)+cos(psi)*cos(theta))+vqz*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi)) ) ;
    z     + T *(-vqx*sin(phi)+vqy*cos(phi)*sin(psi)+vqz*cos(psi)*cos(psi));
    ...
    psi   + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
    phi   + T*( wqy*cos(psi)-wqz*sin(psi));
    theta + T*( wqz*cos(psi)/cos(phi)+wqy*sin(psi)/cos(phi));
    ...
    -(TM(1)*wqx + TM(2)*wqx + TM(3)*wqx + TM(4)*wqx - M*ax*wqx - M*ay*wqy - M*az*wqx + M*g*wqx*sin(phi) - M*g*wqx*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz));
    -(TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqx - M*ay*wqy - M*az*wqz + M*g*wqx*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqz*(wqx - wqz));
    -(TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqz - M*ay*wqy - M*az*wqz + M*g*wqz*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz));
    ...   
    wqx   + T*( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TM(2)-TM(4)) )/Ixx ;
    wqy   + T*( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TM(3)-TM(1)) )/Iyy;
    wqz   + T*( -Izzm*(dw(1)-dw(2)+dw(3)-dw(4))+D(1)-D(2)+D(3)-D(4) )/Izz ...
    ];

h = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM) [ ... % Devuelve [psi;phi;theta;ax;ay;az;wqx;wqy;wqz] --> misma forma que z. Sale tmb del el imu_conv
    psi ; 
    phi ; 
    theta ; 
    vqy*wqz-vqz*wqy+g*sin(phi);                                            % TODO RESTAR G Y VER BIEN Q ES LO Q MIDE EL ACC
    vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi)
    vqx*wqy-vqy*wqz-g*cos(phi)*cos(psi)+1/M*(TM(1)+TM(2)+TM(3)+TM(4));
    wqx ; 
    wqy ; 
    wqz ...
    ];    

F = @(x,y,z,psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,ax,ay,az,w,dw,TM,D) ...
	[ ... 
    1, 0, 0,                          T*(vqz*(cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi)) + vqy*cos(psi)*cos(theta)*sin(phi)), T*(vqy*(sin(phi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)) - vqx*cos(theta)*sin(phi) + vqz*cos(phi)*cos(psi)*cos(theta)), -T*(vqy*(cos(phi)*cos(theta) + sin(phi)*sin(psi)*sin(theta)) - vqz*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + vqx*cos(phi)*sin(theta)), T*cos(phi)*cos(theta), -T*(cos(phi)*sin(theta) - cos(theta)*sin(phi)*sin(psi)),  T*(sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi)),                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                          0;
    0, 1, 0, -T*(vqy*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + vqz*(cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta))),                         T*(vqz*cos(phi)*cos(psi)*sin(theta) - vqx*sin(phi)*sin(theta) + vqy*cos(phi)*sin(psi)*sin(theta)),  T*(vqz*(sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi)) - vqy*(cos(psi)*sin(theta) - cos(theta)*sin(phi)*sin(psi)) + vqx*cos(phi)*cos(theta)), T*cos(phi)*sin(theta),  T*(cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta)), -T*(cos(theta)*sin(psi) - cos(psi)*sin(phi)*sin(theta)),                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                          0;
    0, 0, 1,                                                                      T*(vqy*cos(phi)*cos(psi) - 2*vqz*cos(psi)*sin(psi)),                                                                                 -T*(vqx*cos(phi) + vqy*sin(phi)*sin(psi)),                                                                                                                                                  0,           -T*sin(phi),                                     T*cos(phi)*sin(psi),                                            T*cos(psi)^2,                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                          0;
    0, 0, 0,                                                                    T*(wqy*cos(psi)*tan(phi) - wqz*sin(psi)*tan(phi)) + 1,                                                         T*(wqz*cos(psi)*(tan(phi)^2 + 1) + wqy*sin(psi)*(tan(phi)^2 + 1)),                                                                                                                                                  0,                     0,                                                       0,                                                       0,                                                                                                                                                                                                                                                                                            T,                                                                                                                                                                                                                    T*sin(psi)*tan(phi),                                                                                                                                                                                                                                                                                                                                                                                                                                        T*cos(psi)*tan(phi);
    0, 0, 0,                                                                                         -T*(wqz*cos(psi) + wqy*sin(psi)),                                                                                                                         1,                                                                                                                                                  0,                     0,                                                       0,                                                       0,                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                             T*cos(psi),                                                                                                                                                                                                                                                                                                                                                                                                                                                -T*sin(psi);
    0, 0, 0,                                                                    T*((wqy*cos(psi))/cos(phi) - (wqz*sin(psi))/cos(phi)),                                               T*((wqz*cos(psi)*sin(phi))/cos(phi)^2 + (wqy*sin(phi)*sin(psi))/cos(phi)^2),                                                                                                                                                  1,                     0,                                                       0,                                                       0,                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                  (T*sin(psi))/cos(phi),                                                                                                                                                                                                                                                                                                                                                                                                                                      (T*cos(psi))/cos(phi);
    0, 0, 0,                                              (M*g*wqy*cos(phi)*cos(psi) - M*g*wqx*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)),                           -(M*g*wqx*cos(phi) + M*g*wqx*cos(psi)*sin(phi) + M*g*wqy*sin(phi)*sin(psi))/(M*wqy*(wqx - wqz)),                                                                                                                                                  0,                     0,                                                       0,                                                       0, (TM(1)*wqx + TM(2)*wqx + TM(3)*wqx + TM(4)*wqx - M*ax*wqx - M*ay*wqy - M*az*wqx + M*g*wqx*sin(phi) - M*g*wqx*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)^2) - (TM(1) + TM(2) + TM(3) + TM(4) - M*ax - M*az + M*g*sin(phi) - M*g*cos(phi)*cos(psi))/(M*wqy*(wqx - wqz)), (M*ay + M*g*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)) + (TM(1)*wqx + TM(2)*wqx + TM(3)*wqx + TM(4)*wqx - M*ax*wqx - M*ay*wqy - M*az*wqx + M*g*wqx*sin(phi) - M*g*wqx*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy^2*(wqx - wqz)),                                                                                                                                                                                                                                                                         -(TM(1)*wqx + TM(2)*wqx + TM(3)*wqx + TM(4)*wqx - M*ax*wqx - M*ay*wqy - M*az*wqx + M*g*wqx*sin(phi) - M*g*wqx*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)^2);               
    0, 0, 0,                                              (M*g*wqy*cos(phi)*cos(psi) - M*g*wqz*cos(phi)*sin(psi))/(M*wqz*(wqx - wqz)),                           -(M*g*wqx*cos(phi) + M*g*wqz*cos(psi)*sin(phi) + M*g*wqy*sin(phi)*sin(psi))/(M*wqz*(wqx - wqz)),                                                                                                                                                  0,                     0,                                                       0,                                                       0,                                                                (M*ax - M*g*sin(phi))/(M*wqz*(wqx - wqz)) + (TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqx - M*ay*wqy - M*az*wqz + M*g*wqx*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqz*(wqx - wqz)^2),                                                                                                                                                                                     (M*ay + M*g*cos(phi)*sin(psi))/(M*wqz*(wqx - wqz)), (TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqx - M*ay*wqy - M*az*wqz + M*g*wqx*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqz^2*(wqx - wqz)) - (TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqx - M*ay*wqy - M*az*wqz + M*g*wqx*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqz*(wqx - wqz)^2) - (TM(1) + TM(2) + TM(3) + TM(4) - M*az - M*g*cos(phi)*cos(psi))/(M*wqz*(wqx - wqz));
    0, 0, 0,                                              (M*g*wqy*cos(phi)*cos(psi) - M*g*wqz*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)),                           -(M*g*wqz*cos(phi) + M*g*wqz*cos(psi)*sin(phi) + M*g*wqy*sin(phi)*sin(psi))/(M*wqy*(wqx - wqz)),                                                                                                                                                  0,                     0,                                                       0,                                                       0,                                                                                                            (TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqz - M*ay*wqy - M*az*wqz + M*g*wqz*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)^2), (M*ay + M*g*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)) + (TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqz - M*ay*wqy - M*az*wqz + M*g*wqz*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy^2*(wqx - wqz)),                                                                                                                                                             - (TM(1) + TM(2) + TM(3) + TM(4) - M*ax - M*az + M*g*sin(phi) - M*g*cos(phi)*cos(psi))/(M*wqy*(wqx - wqz)) - (TM(1)*wqz + TM(2)*wqz + TM(3)*wqz + TM(4)*wqz - M*ax*wqz - M*ay*wqy - M*az*wqz + M*g*wqz*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz)^2);        
    0, 0, 0,                                                                                                                        0,                                                                                                                         0,                                                                                                                                                  0,                     0,                                                       0,                                                       0,                                                                                                                                                                                                                                                                                            1,                                                                                                                                                                           (T*(wqz*(Iyy - Izz) + Izzm*(w(1) - w(2) + w(3) - w(4))))/Ixx,                                                                                                                                                                                                                                                                                                                                                                                                                                    (T*wqy*(Iyy - Izz))/Ixx;
    0, 0, 0,                                                                                                                        0,                                                                                                                         0,                                                                                                                                                  0,                     0,                                                       0,                                                       0,                                                                                                                                                                                                                                -(T*(wqz*(Ixx - Izz) - Izzm*(w(1) - w(2) + w(3) - w(4))))/Iyy,                                                                                                                                                                                                                                      1,                                                                                                                                                                                                                                                                                                                                                                                                                                   -(T*wqx*(Ixx - Izz))/Iyy;
    0, 0, 0,                                                                                                                        0,                                                                                                                         0,                                                                                                                                                  0,                     0,                                                       0,                                                       0,                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                           1 ...
    ];
 
H = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz) ...
    [ ...
    0, 0, 0,                    1,                   0, 0,    0,    0,    0,   0,    0,    0;
    0, 0, 0,                    0,                   1, 0,    0,    0,    0,   0,    0,    0;
    0, 0, 0,                    0,                   0, 1,    0,    0,    0,   0,    0,    0;
    0, 0, 0,                    0,          g*cos(phi), 0,    0,  wqz, -wqy,   0, -vqz,  vqy;
    0, 0, 0, -g*cos(phi)*cos(psi), g*sin(phi)*sin(psi), 0, -wqz,    0,  wqx, vqz,    0, -vqx;
    0, 0, 0,  g*cos(phi)*sin(psi), g*cos(psi)*sin(phi), 0,  wqy, -wqz,    0,   0,  vqx, -vqy;
    0, 0, 0,                    0,                   0, 0,    0,    0,    0,   1,    0,    0;
    0, 0, 0,                    0,                   0, 0,    0,    0,    0,   0,    1,    0;
    0, 0, 0,                    0,                   0, 0,    0,    0,    0,   0,    0,    1 ...
    ];

Q = sigma_w^2*eye(Ns);  % 10*eye(Ns)
R = diag(.1*[100 100 100 100 100 100 100 100 100]);        % 1000*eye(Ns)

P = 1*eye(Ns);
x_hat=zeros(N,Ns);
% x_hat(1,:)=[0 0 0 z(1,4) z(1,5) z(1,6) 0 0 0 z(1,7) z(1,8) z(1,9)];

psi_init   = z(1,4);
phi_init   = z(1,5);
theta_init = z(1,6);
wqx_init   = z(1,7);
wqy_init   = z(1,8);
wqz_init   = z(1,9);
vqx_init   = -(TM(1,1)*wqx_init + TM(1,2)*wqx_init + TM(1,3)*wqx_init + TM(1,4)*wqx_init - M*z(1,4)*wqx_init - M*z(1,5)*wqy_init - M*z(1,6)*wqx_init + M*g*wqx_init*sin(phi_init) - M*g*wqx_init*cos(phi_init)*cos(psi_init) - M*g*wqy_init*cos(phi_init)*sin(psi_init))/(M*wqy_init*(wqx_init - wqz_init));
vqy_init   = -(TM(1,1)*wqz_init + TM(1,2)*wqz_init + TM(1,3)*wqz_init + TM(1,4)*wqz_init - M*z(1,4)*wqx_init - M*z(1,5)*wqy_init - M*z(1,6)*wqz_init + M*g*wqx_init*sin(phi_init) - M*g*wqz_init*cos(phi_init)*cos(psi_init) - M*g*wqy_init*cos(phi_init)*sin(psi_init))/(M*wqz_init*(wqx_init - wqz_init));
vqz_init   = -(TM(1,1)*wqz_init + TM(1,2)*wqz_init + TM(1,3)*wqz_init + TM(1,4)*wqz_init - M*z(1,4)*wqz_init - M*z(1,5)*wqy_init - M*z(1,6)*wqz_init + M*g*wqz_init*sin(phi_init) - M*g*wqz_init*cos(phi_init)*cos(psi_init) - M*g*wqy_init*cos(phi_init)*sin(psi_init))/(M*wqy_init*(wqx_init - wqz_init));
xinit = T *(vqx_init*cos(phi_init)*cos(theta_init)+vqy_init*(cos(theta_init)*sin(phi_init)*sin(psi_init)-cos(phi_init)*sin(theta_init))+vqz_init*(sin(psi_init)*sin(theta_init)+cos(psi_init)*cos(theta_init)*sin(phi_init)) );
yinit = T *(vqx_init*cos(phi_init)*sin(theta_init)+vqy_init*(sin(theta_init)*sin(phi_init)*sin(psi_init)+cos(psi_init)*cos(theta_init))+vqz_init*(cos(psi_init)*sin(theta_init)*sin(phi_init)-cos(theta_init)*sin(psi_init)) );
zinit = T *(-vqx_init*sin(phi_init)+vqy_init*cos(phi_init)*sin(psi_init)+vqz_init*cos(psi_init)*cos(psi_init));
x_hat(1,:)=[ xinit yinit zinit psi_init phi_init theta_init vqx_init vqy_init vqz_init wqy_init wqx_init wqz_init];
clear psi_init; clear phi_init; clear theta_init; clear wqx_init; clear wqy_init; clear wqz_init; clear vqx_init; clear vqy_init; clear vqz_init; clear xinit; clear yinit; clear zinit;

for i=2:N
    % Prediction         TODO - agitar que x_ depende de la lectura del acc
    x_   = f(x_hat(i-1,1),x_hat(i-1,2),x_hat(i-1,3),x_hat(i-1,4),x_hat(i-1,5),x_hat(i-1,6),x_hat(i-1,7),x_hat(i-1,8),x_hat(i-1,9),x_hat(i-1,10),x_hat(i-1,11),x_hat(i-1,12),z(i-1,4),z(i-1,5),z(i-1,6),w(i-1,:),dw(i-1,:),TM(i-1,:),D(i-1,:));
    Fk_1 = F(x_hat(i-1,1),x_hat(i-1,2),x_hat(i-1,3),x_hat(i-1,4),x_hat(i-1,5),x_hat(i-1,6),x_hat(i-1,7),x_hat(i-1,8),x_hat(i-1,9),x_hat(i-1,10),x_hat(i-1,11),x_hat(i-1,12),z(i-1,4),z(i-1,5),z(i-1,6),w(i-1,:),dw(i-1,:),TM(i-1,:),D(i-1,:));
    P_   = Fk_1 * P * Fk_1'+ Q; 
    
    % Update
    yk         = z(i,:)' - h(x_(4),x_(5),x_(6),x_(7),x_(8),x_(9),x_(10),x_(11),x_(12),TM(i-1,:));
    Hk         = H(x_(4),x_(5),x_(6),x_(7),x_(8),x_(9),x_(10),x_(11),x_(12));
    Sk         = Hk*P_*Hk' + R;
    Kk         = P_*Hk'*Sk^-1;
    x_hat(i,:) = x_ + Kk*yk;
    P          = (eye(Ns)-Kk*Hk)*P_;
end


%% Plots


figure()
    plot([x_hat(1:end,1)],'b')
    hold on; grid
    plot([x_hat(1:end,2)],'r')
    plot([x_hat(1:end,3)],'g')
    legend('x','y','z')
    hold off    

figure()
    plot(z(:,1),'b--')
    hold on; grid
    plot(z(:,2),'r--')
    plot(z(:,3),'g--')    
    plot([x_hat(1:end,4)],'b')
    plot([x_hat(1:end,5)],'r')
    plot([x_hat(1:end,6)],'g')
    legend('\psi','\phi','\theta','\psi','\phi','\theta')
    hold off    
    
figure()
    plot([x_hat(1:end,7)],'b')
    hold on; grid
    plot([x_hat(1:end,8)],'r')
    plot([x_hat(1:end,9)],'g')
    legend('v_{qx}','v_{qy}','v_{qz}')
    hold off

figure()
    plot(z(:,7),'b--')
    hold on; grid
    plot(z(:,8),'r--')
    plot(z(:,9),'g--')    
    plot([x_hat(1:end,10)],'b')
    plot([x_hat(1:end,11)],'r')
    plot([x_hat(1:end,12)],'g')
    legend('w_x','w_y','w_z','w_x','w_y','w_z')
    hold off
    
figure()
subplot(211)
    plot(z(:,1)-[x_hat(1:end,4)],'b')
    hold on; grid
    plot(z(:,2)-[x_hat(1:end,5)],'r')
    plot(z(:,3)-[x_hat(1:end,6)],'g')    
    legend('\psi','\phi','\theta')
    title('Errores en angulos')
    hold off
subplot(212)
    plot(z(:,7)-[x_hat(1:end,10)],'b')
    hold on; grid
    plot(z(:,8)-[x_hat(1:end,11)],'r')
    plot(z(:,9)-[x_hat(1:end,12)],'g')    
    legend('w_x','w_y','w_z')
    title('Errores en velocidades angulares')
    hold off