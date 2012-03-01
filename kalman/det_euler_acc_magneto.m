close all
clear all
clc
log = 'y00x00';
[acrud,wcrud,mcrud,tcrud,bcrud]=mong_read(['./tests/mongoose/magnetometro/data_horizontal/' log],0);
[a,w,m] = mong_conv(acrud,wcrud,mcrud,0);
alpha = 39.85*pi/180; % inclinacion campo magnetico
ax = mean(a(:,1));
ay = mean(a(:,2));
mx = mean(m(:,1));

%% Valores hallados

phi   = -asin(ax);
psi   = asin(ay/cos(phi));
theta = acos([mx+sin(phi)*sin(alpha)]/[cos(phi)*cos(alpha)]);

fprintf(['---------\nLog ' log '\n---------\nTheta = %f\nPhi = %f\nPsi = %f\n---------\n'],180/pi*theta,180/pi*phi,180/pi*psi)

%% Valores teoricos

