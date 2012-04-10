function [x_hat,P] = kalman_gps(x_hat,P,z)

% -------------------------------------------------------------------------
% function [x_hat,P] = kalman_gps(x_hat,P,z)
% -------------------------------------------------------------------------
% Cosas que estan mal o mas o menos
%   -h y H no deben depender de x_. La solucion para esto es incluir los 
%    angulos de euler en el vecotor de estados de este kalman y no 
%    actualizarlos. No corregirlos con ninguna observacion. Pero me parece 
%    que es lo mismo que hacer lo que esta. Revisarlo por las dudas
% -------------------------------------------------------------------------

%% Constantes

Ns   = 6;       % Largo del vector de estados

Q = diag(1*[100 100 100 100 100 100]);
R = diag(1*[1 1 100000 1 1 100000]);

%% Funciones

Rx = @(psi) [...
    1 0        0        ;
    0 cos(psi) -sin(psi) ;
    0 sin(psi) cos(psi) ...
    ];

Ry = @(phi) [...
    cos(phi)  0 sin(phi)   ;
    0         1     0      ;
    -sin(phi) 0 cos(phi) ...
    ];

Rz = @(theta) [...
    cos(theta) -sin(theta) 0 ;
    sin(theta) cos(theta)  0 ;
    0          0           1 ...
    ];

f = @(x,y,z,vqx,vqy,vqz) [ ...
    x   ;
    y   ;
    z   ;
    vqx ;
    vqy ;
    vqz ...
    ];

h = @(x,y,z,psi,phi,theta,vqx,vqy,vqz) [ ... 
    x ;
    y ;
    z ;
    Rz(theta)*Ry(phi)*Rx(psi)*[vqx;vqy;vqz] ...
    ];

F = @() ...
    eye(Ns);

H = @(psi,phi,theta) [...
    1, 0, 0,                   0,                                                  0,                                                  0 ;
    0, 1, 0,                   0,                                                  0,                                                  0 ;
    0, 0, 1,                   0,                                                  0,                                                  0 ;
    0, 0, 0, cos(phi)*cos(theta), cos(theta)*sin(phi)*sin(psi) - cos(psi)*sin(theta), sin(psi)*sin(theta) + cos(psi)*cos(theta)*sin(phi) ;
    0, 0, 0, cos(phi)*sin(theta), cos(psi)*cos(theta) + sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi)*sin(theta) - cos(theta)*sin(psi) ;
    0, 0, 0,           -sin(phi),                                  cos(phi)*sin(psi),                                  cos(phi)*cos(psi) ...
    ];
 
%% Kalman

% Predict
x_   = f(x_hat(1),x_hat(2),x_hat(3),x_hat(7),x_hat(8),x_hat(9));

Fk_1 = F();

P_   = Fk_1 * P * Fk_1'+ Q; 

% Update
yk         = z - h(x_(1),x_(2),x_(3),x_hat(4),x_hat(5),x_hat(6),x_(4),x_(5),x_(6));
Hk         = H(x_hat(4), x_hat(5), x_hat(6)); % OJO TODO H deberia depender solo de x_
Sk         = Hk*P_*Hk' + R;
Kk         = P_*Hk'*Sk^-1;
x_hat      = x_ + Kk*yk;
P          = (eye(Ns)-Kk*Hk)*P_;