function [x_hat,P] = kalman_gps(x_hat,P,z)

%% Constantes

% Ixx  = 2.32e-2; % Tensor de inercia del quad - según x
% Iyy  = 2.32e-2; % Tensor de inercia del quad - según y
% Izz  = 4.37e-2; % Tensor de inercia del quad - según z
% Izzm = 1.54e-5; % Tensor de inercia de los motores - segun z
% L    = 0.29;    % Largo en metros del los brazos del quad
% M    = 1.541;   % Masa del Quad en kg
% g    = 9.81;    % Aceleracion gravitatoria

Ns   = 3;       % Largo del vector de estados

Q = diag(1*[100 100 100]);
R = diag(100*[100 100 100]);

%% Entradas

% dw = zeros(4);  % Derivada de w. Cada columna corresponde a 1 motor
% TM = drive(u);  % Fuerzas ejercidas por los motores en N. Cada columna corresponde a 1 motor.
% D  = drag(u);   % Torque de Drag ejercido por los motores en N*m. Cada columna corresponde a cada motor

%% Funciones
     
f = @(x,y,z) [ ...
    x ;
    y ;
    z ...
    ];

h = @(x,y,z) [ ... 
    x ;
    y ;
    z ...
    ];

F = @() ...
    eye(Ns);
 
H = @() ...
    eye(Ns);

%% Kalman

% Predict
x_   = f(x_hat(1),x_hat(2),x_hat(3));

Fk_1 = F();

P_   = Fk_1 * P * Fk_1'+ Q; 

% Update
yk         = z - h(x_(1),x_(2),x_(3));
Hk         = H();
Sk         = Hk*P_*Hk' + R;
Kk         = P_*Hk'*Sk^-1;
x_hat      = x_ + Kk*yk;
P          = (eye(Ns)-Kk*Hk)*P_;