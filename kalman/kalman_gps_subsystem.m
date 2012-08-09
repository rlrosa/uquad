function [x_hat,P] = kalman_gps_subsystem(x_hat,P,Q,R,T,z,realim)

% -------------------------------------------------------------------------
% function [x_hat,P] = kalman_gps_subsystem(x_hat,P,Q,R,z)
% -------------------------------------------------------------------------
% vector de estados [x vx ax y vy ay z vz az]
% -------------------------------------------------------------------------

Ns = length(P);

% F = [...
%     1 T T^2/2 0 0 0     0 0 0
%     0 1 T     0 0 0     0 0 0
%     0 0 1     0 0 0     0 0 0
%     0 0 0     1 T T^2/2 0 0 0
%     0 0 0     0 1 T     0 0 0
%     0 0 0     0 0 1     0 0 0
%     0 0 0     0 0 0     1 T T^2/2
%     0 0 0     0 0 0     0 1 T
%     0 0 0     0 0 0     0 0 1
%     ];

F = eye(Ns);

H = [...
    1 0 0 0 0 0 0 0 0
    0 0 0 1 0 0 0 0 0
    0 0 0 0 0 0 1 0 0
    ];

% Predict
x_   = F*x_hat;
P_   = F*P*F'+Q; 

% Update
if realim
    y = z-H*x_;
else
    y = zeros(3,1);
end
S         = H*P_*H'+R;
K         = P_*H'*S^-1;
x_hat     = x_ + K*y;
P         = (eye(Ns)-K*H)*P_;
