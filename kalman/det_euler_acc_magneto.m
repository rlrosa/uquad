function [theta, phi, psi] = det_euler_acc_magneto (a,m,print)

if nargin < 3
    print = 1;
end

% close all
% clear all
% clc
% log = 'y00x00';
% [acrud,wcrud,mcrud,~,~]=mong_read(['./tests/mongoose/magnetometro/data_horizontal/' log],0);
% [a,~,m] = mong_conv(acrud,wcrud,mcrud,0);

am = mean(a);
mm = mean(m);

if (abs(am(1))<9.72)
    phi=-180/pi*asin(am(1)/9.81);
    psi=180/pi*atan2(am(2),am(3));
elseif am(1) > 0
    phi=-90;
    psi=0;
else 
    phi=90;
    psi=0;
end

mrot = [ cosd(phi)/(cosd(phi)^2 + sind(phi)^2), (sind(phi)*sind(psi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2)), (cosd(psi)*sind(phi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2));
                                             0,                                           cosd(psi)/(cosd(psi)^2 + sind(psi)^2),                                         -sind(psi)/(cosd(psi)^2 + sind(psi)^2);
        -sind(phi)/(cosd(phi)^2 + sind(phi)^2), (cosd(phi)*sind(psi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2)), (cosd(phi)*cosd(psi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2))]...
        *mm';

theta=180/pi*atan2(mrot(1),mrot(2))+9.78;    

if print
    rayas = '-------------------\n';
    fprintf([rayas 'Theta = %f\nPhi = %f\nPsi = %f\n' rayas],theta,phi,psi);
end




























% % Ojo con los angulos hallados - me quedo solo con la parte real, y eso no
% % se si esta del todo bien
% 
% close all
% clear all
% clc
% log = 'z00y00';
% [acrud,wcrud,mcrud,tcrud,bcrud]=mong_read(['./tests/mongoose/magnetometro/data_horizontal/' log],0);
% [a,w,m] = mong_conv(acrud,wcrud,mcrud,0);
% alpha = 39.85; % inclinacion campo magnetico en º
% 
% rayas = '------------------\n';
% 
% ax = mean(a(:,1))/9.81;
% ay = mean(a(:,2))/9.81;
% az = mean(a(:,3))/9.81;
% mx = mean(m(:,1));
% my = mean(m(:,2));
% mz = mean(m(:,3));
% 
% %% Metodo 1
% 
% pitch = real(-asind(ax));
% roll  = real(asind(ay/cosd(pitch)));
% yaw   = real(acosd([mx+sind(pitch)*sind(alpha)]/[cosd(pitch)*cosd(alpha)]));
% 
% % fprintf([rayas Log ' log ' rayas 'Metodo paper\n' rayas 'Theta = %f\nPhi = %f\nPsi = %f\n---------\n'],yaw,pitch,roll)
% fprintf(['Metodo ellos\n---------\nTheta = %f\nPhi = %f\nPsi = %f\n---------\n'],yaw,pitch,roll)
% 
% %% cualquiera?
% 
% pitch = (-asind(ax/9.81));
% roll  = 180/pi*(-atan2(ay,az));
% yaw   = (acosd([mx+sind(pitch)*sind(alpha)]/[cosd(pitch)*cosd(alpha)]));
% 
% fprintf(['Metodo yo\n---------\nTheta = %f\nPhi = %f\nPsi = %f\n---------\n'],yaw,pitch,roll)
% 
% % En nuestra notacion
% 
% theta = yaw;
% phi   = pitch;
% psi   = roll;
% 
% 
% %% Metodo 2
% break
% phi = real(atand(ay/az));
% theta = real(acosd(ay/sin(phi)));
% 
% A = sind(phi)*sind(theta)*cosd(alpha);
% B = cosd(phi)*cosd(alpha);
% C = sind(phi)*cosd(theta)*sind(alpha);
% D = cosd(phi)*cosd(theta)*sind(alpha);
% E = cosd(phi)*sind(theta)*cosd(alpha);
% F = sind(phi)*cosd(alpha);
% 
% psi = real(asin((A*(mz-D)-E*(my-C))/(B*E+A*F)));
% 
% fprintf('Metodo 2\n---------\nTheta = %f\nPhi = %f\nPsi = %f\n---------\n',theta,phi,psi)
