close all
clear all
clc

%% Constantes

f = 100; % Frecuencia en Hertz
T = 1/f; % Período
str='bgrcmy';
prueba = '1';

%% Calib

[a,w] = imu_read(['z',prueba,'.txt']);
az_z=a(:,3);
[a,w] = imu_read(['y',prueba,'.txt']);
az_y=a(:,3);
[a,w] = imu_read(['x',prueba,'.txt']);
az_x=a(:,3);

az = [az_z;az_y;az_x];

A = [az ones(length(az),1)];
B = [-9.81*ones(size(az_z,1),1);zeros(size(az_y,1)+size(az_x,1),1)];

G=A\B;
b=G(2)
G=G(1)

%% Integracion con el 'y' pa arriba - Velocidad

teo=G*az_y+b;
vz_y = zeros(size(az_y));
vz_y(1) =teo(1)*T;
for i=2:length(az_y)
    vz_y(i)=vz_y(i-1)+teo(i)*T;
end

figure
plot(vz_y)

%% Integracion con el 'y' pa arriba - Posición

pz_y = zeros(size(az_y));
pz_y(1) =vz_y(1)*T;
for i=2:length(az_y)
    pz_y(i)=pz_y(i-1)+vz_y(i)*T;
end

figure
plot(pz_y)




% %% Promediando aceleracion de a 10 muestras
% 
% c=1;
% for i=1:10:length(az_y)-9
%     medias(c)=mean(az_y(i:i+9));
%     c=c+1;
% end