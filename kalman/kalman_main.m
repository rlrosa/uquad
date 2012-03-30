% close all
% clear all
% clc

%% Load data

% Imu
% imu_file = '../../../Escritorio/imu_raw.log';
% imu_file = 'tests/main/logs/vuelo_5/imu_raw.log';
imu_file = 'tests/main/logs/29marzo/imu_raw.log';
[acrud,wcrud,mcrud,~,bcrud,~,~,T]=mong_read(imu_file,0,1);
avg = 24;
acrud(:,1) = moving_avg(acrud(:,1),avg); acrud(:,2) = moving_avg(acrud(:,2),avg); acrud(:,3) = moving_avg(acrud(:,3),avg);
wcrud(:,1) = moving_avg(wcrud(:,1),avg); wcrud(:,2) = moving_avg(wcrud(:,2),avg); wcrud(:,3) = moving_avg(wcrud(:,3),avg);
mcrud(:,1) = moving_avg(mcrud(:,1),avg); mcrud(:,2) = moving_avg(mcrud(:,2),avg); mcrud(:,3) = moving_avg(mcrud(:,3),avg);
[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0);
b=altitud(bcrud);

% % Gps
% % gps_file = '~/Escritorio/car/01.log';
% % [easting, northing, elevation, utmzone, sat, lat, lon, dop] = ...
% %     gpxlogger_xml_handler(gps_file, 1);
% % save('kalman/gps','easting','northing','elevation','utmzone','sat','lat','lon','dop');
% GPS = load('kalman/gps.mat');
% easting = GPS.easting; northing = GPS.northing; elevation = GPS.elevation; 
% utmzone = GPS.utmzone; sat = GPS.sat; lat = GPS.lat; lon = GPS.lon; dop = GPS.dop;

%% Constantes, entradas, observaciones e inicialización

% Constantes
N       = size(a,1);         % Cantidad de muestras de las observaciones
Ns      = 12;                % N states: cantidad de variables de estado
w_hover = 298.0867;
w_max   = 387.0; 
w_min   = 109.0; 
K       = load('K.mat');K = K.K;
sp_x    = zeros(7,1);
sp_w    = ones(4,1)*w_hover;

% Entradas
dw      = zeros(N,4);        % Derivada de w. Cada columna corresponde a 1 motor
TM      = drive(w);          % Fuerzas ejercidas por los motores en N. Cada columna corresponde a 1 motor.
D       = drag(w);           % Torque de Drag ejercido por los motores en N*m. Cada columna corresponde a cada motor

% Observaciones
z  = [euler a w b];     

% Inicialización
x_hat         = zeros(N,Ns);
P             = 1*eye(Ns);
w_control     = zeros(N,4);
x_hat_partial = zeros(N,7);

%% Kalman

for i=2:N
    % Kalman
    [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,T(i),w_control(i-1,:)',z(i,:)');
   
%     if gps_available
%         [aux,P_gps]    = kalman_gps(x_hat(i-1,1:3),P_gps,[easting(i) northing(i) elevation(i)]);
%         x_hat(i-1,1:3) = aux;
%     end
    
    % Control
    x_hat_partial(i,:) = [x_hat(i,3), x_hat(i,4), x_hat(i,5), x_hat(i,9), ...
        x_hat(i,10), x_hat(i,11), x_hat(i,12)];
    w_control(i,:) = (sp_w + K*(sp_x - x_hat_partial(i,:)'))';
    for j=1:4
      if(w_control(i,j) < w_min)
        w_control(i,j) = w_min;
      end
      if (w_control(i,j) > w_max)
        w_control(i,j) = w_max;
      end
    end    
end

%% Plots

figure()
subplot(221)
    plot([x_hat(1:end,1)],'b')
    hold on; grid
    plot([x_hat(1:end,2)],'r')
    plot(b,'k')
    plot([x_hat(1:end,3)],'g')
    legend('x','y','z')
    hold off    

subplot(222)
    plot(180/pi*z(:,1),'k')
    hold on; grid
    plot(180/pi*z(:,2),'k')
    plot(180/pi*z(:,3),'k')    
    plot(180/pi*[x_hat(1:end,4)],'b')
    plot(180/pi*[x_hat(1:end,5)],'r')
    plot(180/pi*[x_hat(1:end,6)],'g')
    legend('\psi','\phi','\theta','\psi','\phi','\theta')
    hold off    
    
subplot(223)
    plot([x_hat(1:end,7)],'b')
    hold on; grid
    plot([x_hat(1:end,8)],'r')
    plot([x_hat(1:end,9)],'g')
    legend('v_{qx}','v_{qy}','v_{qz}')
    hold off

subplot(224)
    plot(z(:,7),'k')
    hold on; grid
    plot(z(:,8),'k')
    plot(z(:,9),'k')    
    plot([x_hat(1:end,10)],'b')
    plot([x_hat(1:end,11)],'r')
    plot([x_hat(1:end,12)],'g')
    legend('w_x','w_y','w_z','w_x','w_y','w_z')
    hold off
%%
figure()
    plot(w_control)
    axis([2 length(w_control) min(min(w_control(2:end,:))) ...
        max(max(w_control(2:end,:)))])
    title('w motores')
    legend('adelante','izquierda','atras','derecha')