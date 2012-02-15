close all
clear all
clc

%% Constantes

f = 100; % Frecuencia en Hertz
T = 1/f; % Período
str='bgrcmy';

%% Datos

[a,w] = imu_read('z1.txt');
Nn    = size(a,1);
ang = w*T;

%% Init

R  = zeros (3,3,Nn);
B  = zeros (3,6,Nn);
B0 = [ 0 1 0 0 0 0  ;
       0 0 0 1 0 0  ;
       0 0 0 0 0 1 ];
   
R(:,:,1) = SpinCalc('EA123toDCM',[180/pi*ang(1,1),180/pi*ang(1,2),180/pi*ang(1,3)],1,0);
B(:,:,1) = R(:,:,1)*B0;
    
%% Posiciones

for i=2:Nn
    roll     = ang(i,1);
    pitch    = ang(i,2);
    yaw      = ang(i,3);    
    R(:,:,i) = SpinCalc('EA123toDCM',[180/pi*roll,180/pi*pitch,180/pi*yaw],1,0);
    B(:,:,i) = R(:,:,i)*B(:,:,i-1);
end

%% Plots

figure()
    plot3(B0(2,:),B0(1,:),B0(3,:),'b-','linewidth',2)
    text(B0(2,2),B0(1,2),B0(3,2),'x','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
    text(B0(2,4),B0(1,4),B0(3,4),'y','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
    text(B0(2,6),B0(1,6),B0(3,6),'z','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
    hold on
    set(gca,'ZDir','reverse')
    for i=50:60
        plot3(B(2,:,i),B(1,:,i),B(3,:,i),[str(rem(i,6)+1) '-'],'linewidth',1.5)
        text(B(2,2,i),B(1,2,i),B(3,2,i),['x_{' num2str(i) '}'],'FontAngle','italic','FontSize',8)
        text(B(2,4,i),B(1,4,i),B(3,4,i),['y_{' num2str(i) '}'],'FontAngle','italic','FontSize',8)
        text(B(2,6,i),B(1,6,i),B(3,6,i),['z_{' num2str(i) '}'],'FontAngle','italic','FontSize',8)
    end





%% Cosas que no uso pero estan buenas

% %% Definición de matrices de rotación
% 
% R1=[ 1     0         0       ;  
%      0  cos(roll) sin(roll)  ;
%      0 -sin(roll) cos(roll) ];
%  
% R2=[ cos(pitch) 0 -sin(pitch) ; 
%          0      1     0      ;
%      sin(pitch) 0  cos(pitch)];
% 
% R3=[  cos(yaw) sin(yaw)   0   ;
%      -sin(yaw) cos(yaw)   0   ;
%          0        0       1  ];
%      
% % R=R1*R2*R3;
% 
% %% Plots
% 
% figure()    
%         plot3(BASE(2,:),BASE(1,:),BASE(3,:),'b-','linewidth',2);
%     text(BASE(2,2),BASE(1,2),BASE(3,2),'x','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
%     text(BASE(2,4),BASE(1,4),BASE(3,4),'y','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
%     text(BASE(2,6),BASE(1,6),BASE(3,6),'z','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
%     hold on
%     set(gca,'ZDir','reverse')
%         plot3(BASE_R(2,:),BASE_R(1,:),BASE_R(3,:),'r-','linewidth',2);
%     text(BASE_R(2,2),BASE_R(1,2),BASE_R(3,2),'x_r','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
%     text(BASE_R(2,4),BASE_R(1,4),BASE_R(3,4),'y_r','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')
%     text(BASE_R(2,6),BASE_R(1,6),BASE_R(3,6),'z_r','FontAngle','italic','FontSize',18,'VerticalAlignment','bottom')    
%     
% disp(['Ángulos en grados' char(10) '[roll pitch yaw] = ' mat2str(SpinCalc('DCMtoEA123',R,1,0))]);