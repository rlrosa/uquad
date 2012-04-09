% -------------------------------------------------------------------------
% Modelo fisico
% -------------------------------------------------------------------------
% d(x)/dt     = vqx*cos(phi)*cos(theta)+vqy*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vqz*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi))
% d(y)/dt     = vqx*cos(phi)*sin(theta)+vqy*(sin(theta)*sin(phi)*sin(psi)+cos(psi)*cos(theta))+vqz*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi))
% d(z)/dt     = -vqx*sin(phi)+vqy*cos(phi)*sin(psi)+vqz*cos(psi)*cos(psi)
% d(psi)/dt   = wqx+wqz*tan(fi)*cos(psi)+wqy*tan(fi)*sin(psi);
% d(phi)/dt   = wqy*cos(psi)-wqz*sin(psi);
% d(theta)/dt = wqz*cos(psi)/cos(fi)+wqy*sin(psi)/cos(fi);
% d(vqx)/dt   = vqy*wqz-vqz*wqy+g*sin(phi)
% d(vqy)/dt   = vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi)
% d(vqz)/dt   = vqx*wqy-vqy*wqx-g*cos(phi)*cos(psi)+1/M*(TM(1)+TM(2)+TM(3)+TM(4))
% d(wqx)/dt   = ( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w1-w2+w3-w4)+L*(T2-T4) )/Ixx;
% d(wqy)/dt   = ( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w1-w2+w3-w4)+L*(T3-T1) )/Iyy;
% d(wqz)/dt   = ( -Izzm*(dw1-dw2+dw3-dw4)+Q1-Q2+Q3-Q4 )/Izz;
% 
% -------------------------------------------------------------------------
% Estado
% -------------------------------------------------------------------------
% x = [x y z psi phi tehta vqx vqy vqz wqx wqy wqz]
% 
% -------------------------------------------------------------------------
% Kalman
% -------------------------------------------------------------------------
% A partir de los datos del gyro se hace un Kalman para la estimación de la
% velocidad angular en las 3 direcciones. A partir de los datos del
% magnetometro, convertidos a angulos de euler en mong_conv se estiman los
% angulos con el mismo filtro de kalman.
% A su vez con los datos de los acelerometros y los otros estados estimados
% se hallan las velocidades referenciadas al quad (vq) y por ultimo se
% incluye la estimacion de la posicion absoluta del quad, utilizando
% unicamente el modelo fisico. No se realiza correcion con ningun sensor a
% los estados x, y, z (posicion absoluta)
% -------------------------------------------------------------------------

%% Load data

% Imu
imu_file = 'tests/main/logs/2012_04_06_1_1_izquierda/imu_raw.log';
[acrud,wcrud,mcrud,~,bcrud,~,~,T]=mong_read(imu_file,0,1);
avg = 1;
startup_runs = 200;
imu_calib = 512;
kalman_startup = 200;

% startup_runs samples are discarded
acrud = acrud(startup_runs:end,:);
wcrud = wcrud(startup_runs:end,:);
mcrud = mcrud(startup_runs:end,:);
bcrud = bcrud(startup_runs:end,:);
T = T(startup_runs:end);

% p0 and theta0 is estimated form first imu_calib samples
[a_calib,w_calib,euler_calib] = mong_conv(acrud(1:imu_calib,:),wcrud(1:imu_calib,:),mcrud(1:imu_calib,:),0);
theta0                  = mean(euler_calib(:,3));
b0                      = mean(bcrud(1:imu_calib));

% averages are used
acrud(:,1) = moving_avg(acrud(:,1),avg); acrud(:,2) = moving_avg(acrud(:,2),avg); acrud(:,3) = moving_avg(acrud(:,3),avg);
wcrud(:,1) = moving_avg(wcrud(:,1),avg); wcrud(:,2) = moving_avg(wcrud(:,2),avg); wcrud(:,3) = moving_avg(wcrud(:,3),avg);
mcrud(:,1) = moving_avg(mcrud(:,1),avg); mcrud(:,2) = moving_avg(mcrud(:,2),avg); mcrud(:,3) = moving_avg(mcrud(:,3),avg);
bcrud      = moving_avg(bcrud,avg);

% first imu_calib values are not used for kalman/control/etc
acrud = acrud(imu_calib:end,:);
wcrud = wcrud(imu_calib:end,:);
mcrud = mcrud(imu_calib:end,:);
bcrud = bcrud(imu_calib:end,:);
T     = T(imu_calib:end,:);

[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0);
b=altitud(bcrud,b0);

% gyro offset comp
w_calib_mean = mean(w_calib);
for i = 1:3
  w(:,i) = w(:,i) - w_calib_mean(i);
end

% Gps
% gps_file = '~/Escritorio/car/01.log';
% [easting, northing, elevation, utmzone, sat, lat, lon, dop] = ...
%     gpxlogger_xml_handler(gps_file, 1);
% save('kalman/gps','easting','northing','elevation','utmzone','sat','lat','lon','dop');
GPS       = load('kalman/gps.mat');
easting   = GPS.easting; northing = GPS.northing; elevation = GPS.elevation; 
utmzone   = GPS.utmzone; sat = GPS.sat; lat = GPS.lat; lon = GPS.lon; dop = GPS.dop;
easting   = easting - mean(easting);
northing  = northing - mean(northing);
elevation = elevation - mean(elevation);

%% Constantes, entradas, observaciones e inicialización

% Constantes
N       = size(a,1);         % Cantidad de muestras de las observaciones
Ns      = 12;                % N states: cantidad de variables de estado
Ngps    = 3;                 % N gps: cantidad de variables corregidas por gps
Nc      = 8;                 % N control states: Cantidad de estados a controlar
% w_idle  = 109;               %     se controla z,psi,phi,theta,vqz,wqx,wqy,wqz
w_hover = 316.10;
w_max   = 387.0; 
w_min   = 109.0; 
K       = load('K.mat');K = K.K;
sp_x    = [0;0;0;theta0;0;0;0;0];
sp_w    = ones(4,1)*w_hover;

% Entradas
dw      = zeros(N,4);        % Derivada de w. Cada columna corresponde a 1 motor
TM      = drive(w);          % Fuerzas ejercidas por los motores en N. Cada columna corresponde a 1 motor.
D       = drag(w);           % Torque de Drag ejercido por los motores en N*m. Cada columna corresponde a cada motor

% Observaciones
z  = [euler a w b];     

% Inicialización
x_hat          = zeros(N,Ns);
P              = 1*eye(Ns);
w_control      = zeros(N-kalman_startup,4);
w_control(1,:) = w_hover*ones(size(w_control(1,:)));
x_hat_partial  = zeros(N,8);

P_gps          = 1*eye(Ngps);
gps_count      = 0;
gps_index      = 2;

%% Kalman

for i=2:N
    wc_i = i-kalman_startup;
    gps_count = gps_count + 1;

    % Kalman
    if(i > kalman_startup + 1)
      % Use control output as current w
      [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,T(i)-T(i-1),w_control(wc_i - 1,:)',z(i,:)');
    else
      % Use set point w as current w
      [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,T(i)-T(i-1),sp_w,z(i,:)');
    end
   
    if gps_count == 100
        [aux,P_gps]    = kalman_gps(x_hat(gps_index-1,1:3),P_gps,[northing(gps_index);easting(gps_index);elevation(gps_index)]);
        x_hat(gps_index-1,1:3) = aux;
        gps_index = gps_index + 1;
        gps_count = 0;
    end
    
    % Control
    x_hat_partial(i,:) = [x_hat(i,3), x_hat(i,4), x_hat(i,5), x_hat(i,6), ...
        x_hat(i,9), x_hat(i,10), x_hat(i,11), x_hat(i,12)];

    % First kalman_startup samples will not be used for control
    if~(i > kalman_startup + 1)
      continue;
    end
    w_control(wc_i,:) = (sp_w + K*(sp_x - x_hat_partial(i,:)'))';
    for j=1:4
      if(w_control(wc_i,j) < w_min)
        w_control(wc_i,j) = w_min;
      end
      if (w_control(wc_i,j) > w_max)
        w_control(wc_i,j) = w_max;
      end
    end    
end

%% Plots

% figure; 
%     plot(w_control(:,1)+w_control(:,3)-w_control(:,2)-w_control(:,4),'r','linewidth',3); 
%     title('diferencia entre velocidades angulares (adelante+atras)-(derecha+izquierda)'); 
%     legend('Giro en z')

plot_main(x_hat,T,z,T);
plot_w([T(kalman_startup+1:end) w_control]);
