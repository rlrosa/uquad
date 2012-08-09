close all
clear all
clc

g       = 9.81;
fs      = 100;
T       = 1/fs;
Ixx     = 2.32e-2;
Iyy     = 2.32e-2;
Izz     = 4.37e-2;
Izzm    = 1.54e-5;
L       = 0.29;
masa    = 1.741-0.091;
w_hover = calc_omega(g*masa/4);

w2 = w_hover+15;
w4 = w_hover-15;


asdf=load('kalman/w_control_prueba_sistema_chico','a');
w = asdf.a;
N = length(w);
% N = 1000;
% w  = [zeros(N,1) ones(N,1)*w2 zeros(N,1) ones(N,1)*w4];

TM = drive(w);

% phi = 0;
% wqy = 0;
% wqz = 0;

psi = zeros(N,1);
wqx = zeros(N,1);

% f = @(psi,phi,wqx,wqy,wqz,w,TM) ...
%     [
%     psi + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
%     wqx + T*( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w(1)- w(2)+w(3)-w(4))+L* ...
%     (TM(2)-TM(4)) )/Ixx
%     ];

f = @(psi,wqx,TM) ...
    [
    psi + T*wqx;
    wqx + T*L*(TM(2)-TM(4))/Ixx
    ];

for i=2:N
%     aux = f(psi(i-1),phi,wqx(i-1),wqy,wqz,w(i,:),TM(i,:));
    aux = f(psi(i-1),wqx(i-1),TM(i,:));
    psi(i) = aux(1);
    wqx(i) = aux(2);
end

figure
    plot(psi); legend('psi')
figure
    plot(wqx); legend('wqx')
figure
    plot(w(:,2)); hold on; plot(w(:,4),'r'); legend('izquierda','derecha');