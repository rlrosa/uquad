function kalman_main(imu_file)

%% Load data

% Imu
% imu_file = '../../../Escritorio/imu_raw.log';
% imu_file = 'tests/main/logs/vuelo_5/imu_raw.log';
imu_file = 'tests/main/logs/29marzo/imu_raw.log';
[acrud,wcrud,mcrud,~,bcrud,~,~,T]=mong_read(imu_file,0,1);
avg = 24;
startup_runs = 200;
imu_calib = 512;
kalman_startup = 200;

% startup_runs samples are discarded
acrud = acrud(startup_runs:end,:);
wcrud = wcrud(startup_runs:end,:);
mcrud = mcrud(startup_runs:end,:);
bcrud = bcrud(startup_runs:end,:);
T = T(startup_runs:end,:);

% p0 is estimated form first imu_calib samples
b0 = mean(bcrud(1:imu_calib));

% averages are used
acrud(:,1) = moving_avg(acrud(:,1),avg); acrud(:,2) = moving_avg(acrud(:,2),avg); acrud(:,3) = moving_avg(acrud(:,3),avg);
wcrud(:,1) = moving_avg(wcrud(:,1),avg); wcrud(:,2) = moving_avg(wcrud(:,2),avg); wcrud(:,3) = moving_avg(wcrud(:,3),avg);
mcrud(:,1) = moving_avg(mcrud(:,1),avg); mcrud(:,2) = moving_avg(mcrud(:,2),avg); mcrud(:,3) = moving_avg(mcrud(:,3),avg);

% first imu_calib values are not used for kalman/control/etc
acrud = acrud(imu_calib:end,:);
wcrud = wcrud(imu_calib:end,:);
mcrud = mcrud(imu_calib:end,:);
bcrud = bcrud(imu_calib:end,:);
T     = T(imu_calib:end,:);

[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0);
b=altitud(bcrud,b0);

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
w_control     = zeros(N-kalman_startup,4);
x_hat_partial = zeros(N,7);

%% Kalman

for i=2:N
    wc_i = i-kalman_startup;

    % Kalman
    if(i > kalman_startup + 1)
      % Use control output as current w
      [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,T(i),w_control(wc_i - 1,:)',z(i,:)');
    else
      % Use set point w as current w
      [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,T(i),sp_w,z(i,:)');
    end
   
%     if gps_available
%         [aux,P_gps]    = kalman_gps(x_hat(i-1,1:3),P_gps,[easting(i) northing(i) elevation(i)]);
%         x_hat(i-1,1:3) = aux;
%     end
    
    % Control
    x_hat_partial(i,:) = [x_hat(i,3), x_hat(i,4), x_hat(i,5), x_hat(i,9), ...
        x_hat(i,10), x_hat(i,11), x_hat(i,12)];

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

plot_main(x_hat,z);
plot_w(w_control);