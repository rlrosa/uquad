clear all;
close all;
clc

%% Cargo una serie de datos

Ts = 1e-2; % frecuencia de muestreo

%datos_crudos = load('test_control/logs/quieto');
%[aconv,wconv,euler] = mong_conv(datos_crudos(:,4:6),datos_crudos(:,7:9),datos_crudos(:,10:12),0);
[a,w,m] = mong_read('test_control/logs/quieto2');
[aconv,wconv,euler] = mong_conv(a,w,m,0);

N = 10000; %Moving average size
% t=[0:Ts:(length(euler(:,4)-1)*Ts];

%% Ruido de Roll

autocorr(euler(:,1));
title('');

noise_roll = 180/pi*[mean(euler(:,1)) std(euler(:,1))]; 

%% Ruido de Pitch

autocorr(euler(:,2));
title('');

noise_pitch = 180/pi*[mean(euler(:,2)) std(euler(:,2))]; 

%% Ruido de Yaw

autocorr(euler(:,3));


