clear all;
close all;
clc

%% Cargo una serie de datos

Ts = 1e-2; % frecuencia de muestreo
datos_crudos = load('test_control/pater_data/imu_avg.log');

ax = datos_crudos(:,4); ay = datos_crudos(:,5); az = datos_crudos(:,6);
wx = datos_crudos(:,7); wy = datos_crudos(:,8); wz = datos_crudos(:,9);

mm = datos_crudos(:,10:12);

psi = zeros(length(ax),1);
phi = zeros(length(ax),1);
theta = zeros(length(ax),1);

t=[0:Ts:(length(ax)-1)*Ts];
%% Calcula los ángulos de Euler

for i=1:length(ax)

    if (abs(ax(i))<9.72)
        phi(i)=-180/pi*asin(ax(i)/9.81);
        psi(i)=180/pi*atan2(ay(i),az(i));
    elseif ax(i) > 0
        phi(i)=-90;
        psi(i)=0;
    else 
        phi(i)=90;
        psi(i)=0;
    end
    
    mrot = [ cosd(phi(i)), (sind(phi(i))*sind(psi(i))), cosd(psi(i))*sind(phi(i));
                0, cosd(psi(i)),         -sind(psi(i));
         -sind(phi(i)), cosd(phi(i))*sind(psi(i)), cosd(phi(i))*cosd(psi(i))]...
            *mm(i,:)';

    theta(i)=180/pi*atan2(mrot(1),mrot(2))+9.78;    

end

%% Graficia y calcula las autocorrelaciones de los ángulos

figure;
autocorr(psi)
xlabel('Distancia entre muestras')
ylabel('Autocorrelación de las muestras del ángulo de Roll')
title('')

figure;
autocorr(phi)
xlabel('Distancia entre muestras')
ylabel('Autocorrelación de las muestras del ángulo de Pitch')
title('')

figure;
autocorr(theta)
xlabel('Distancia entre muestras')
ylabel('Autocorrelación de las muestras del ángulo de Yaw')
title('')