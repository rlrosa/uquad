close all
clear all
clc

[acrud,wcrud,ccrud,tcrud,bcrud]=mong_read('/gyro/logs/x00y00',0);
[a,w] = mong_conv(acrud,wcrud,0);
z = w;
N = size(a,1);

Ixx  = 4;
Iyy  = 6;
Izz  = 10;
Izzm = 2;
L    = 0.3;
Q1   = 0.1*ones(N,1);
Q2   = 0.1*ones(N,1);
Q3   = 0.1*ones(N,1);
Q4   = 0.1*ones(N,1);

Q   = [Q1 Q2 Q3 Q4];

w1  = 10*ones(N,1);
w2  = 10*ones(N,1);
w3  = 10*ones(N,1);
w4  = 10*ones(N,1);


w   = [w1 w2 w3 w4];


dw1 = zeros(N,1);
dw2 = zeros(N,1);
dw3 = zeros(N,1);
dw4 = zeros(N,1);

dw  = [dw1 dw2 dw3 dw4];

T1 = 3.5296e-5*w1.^2-4.9293e-4.*w1;
T2 = 3.5296e-5*w2.^2-4.9293e-4.*w2;
T3 = 3.5296e-5*w3.^2-4.9293e-4.*w3;
T4 = 3.5296e-5*w4.^2-4.9293e-4.*w4;

TT = [T1 T2 T3 T4];





s_v         = 0.1;           % state input std
s_n         = 100;           % observation std
% T           = 100;
T           = N;
x           = zeros(3,T);  % state
y           = zeros(3,T);  % observation
f           = @(x,w,TT,dw,v) [(x(2)*x(3)*(Iyy-Izz)+x(2)*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TT(2)-TT(4)) )/Ixx + v; ... 
                        (x(1)*x(3)*(Izz-Ixx)+x(1)*Izzm*(w(1)-w(2)+w(3)-w(4))+L*(TT(3)-TT(1)) )/Iyy + v; ...
                        (-Izzm*(dw(1)-dw(2)+dw(3)-dw(4))+Q(1)-Q(2)+Q(3)-Q(4) )/Izz + v];          % state equation
h           = @(x,n)   x + n;                               % observation equation
x_          = 0;                                                  % state initialization   
lambda = sqrt(2)/s_n;
for t=2:T,
   x(:,t)     = f(x(:,t-1),w(t-1,:),TT(t-1,:),dw(t-1,:),s_v*randn);                                  % Gaussian noise
   x_       = x(t);
   y(t)     = h(x_,s_n*randn);                                    % Gaussian noise
   %y(t)     = h(x_,sign(randn).*(1/lambda).*log(rand));          % Laplace noise
  
end;

%----------------
% EKF
%----------------
x_EKF       = zeros(3,T);                               % estimated state
F           = @(x) [ 0 (x(3)*(Iyy-Izz)+Izzm*(w(1)-w(2)+w(3)-w(4)) )/Ixx ( x(2)*(Iyy-Izz))/Ixx ; ...
           (x(3)*(Izz-Ixx)+Izzm*(w(1)-w(2)+w(3)-w(4)) )/Iyy 0 ( x(1)*(Izz-Ixx))/Iyy ;
           0 0 0 ];       % df/dx

H           = @(x) eye(3);                                % dh/dx
Pf          = eye(3);                                        % initial filter error covariance
xf          = zeros(3,1);
for t=2:T,
   xp       = f(xf,w(t-1,:),TT(t-1,:),dw(t-1,:),s_v*randn);                               % predicted value t-1-->t
   Pp       = F(xf)*Pf*F(xf)' + s_v^2;                 % prediction error covariance
   dh       = H(xp);
   K        = Pp*dh' / (dh*Pp*dh'+s_n^2);              % variance prediction error
   x_EKF(:,t) = xp + K*(y(t)-h(xp,0));                   % filtered value at time
   xf       = x_EKF(:,t);
   Pf       = Pp - K*dh*Pp;                            % variance filtering error
end;


n_visu   = N;
% observation and state
subplot(2,1,1)
plot(1:n_visu,x(1:n_visu),'r:','LineWidth',3)
hold on
plot(1:n_visu,y(1:n_visu),'g')
legend('state x','obs. y')
hold off
title('state and data')
% axis([1 120 -20 20])
% EKF
subplot(2,1,2)
plot(x(1:n_visu),'r:','LineWidth',3)
hold on
plot(x_EKF(1:n_visu),'m','LineWidth',1)
legend('x','x_{EKF}')
title('EKF')
% axis([1 120 -20 20])