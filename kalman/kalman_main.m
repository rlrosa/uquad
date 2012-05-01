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

%% Config

use_n_states = 2; % Regulates number of variables to control. Can be:
                    % 0: uses 8 states      -> [z psi phi tehta vqz wqx wqy wqz]
                    % 1: uses 8 states and their integrals
                    % 2: uses all 12 states -> [x y z psi phi tehta vqx vqy vqz wqx wqy wqz]
                    % 3: uses all 12 states and their integrals
use_gps      = 1; % Use kalman_gps
use_fake_gps = 1; % Feed kalman_gps with fake data (only if use_gps)
use_fake_T   = 0; % Ignore real timestamps from log, use average
allin1       = 1; % Includes inertial and gps Kalman all in 1 filter
use_gps_vel  = 0; % Uses velocity from GPS. Solo funca si allin1==0 (por ahora)
stabilize_ts = 0; % Read out IMU data until stable Ts (matches main.c after 2012-04027)
ctrl_ramp    = 0; % Run kalman+control when ramping motors.

%% Sanity check
if(use_fake_gps && ~use_gps)
  error('Cannot use_fake_gps if use_gps is disabled!');
end
if(use_n_states > 1 && ~use_gps)
  error('Full control should not be performed without GPS!');
end
if(use_n_states < 2 && use_gps)
  warning('GPS data is being ignored, must control all states!');
end
if(use_n_states < 2 && allin1)
  error('Cannot use intertial+GPS kalman without allin1!');
end
%% Source
 log_path = 'src/build/main/';
if(~exist('log_path','var'))
  error('Must define a variable log_path to read from!');
end
imu_file  = [log_path '/imu_raw.log'];
gps_file  = [log_path '/gps.log'];

%% Load IMU data

% Imu
imu_file = 'tests/main/logs/';
% imu_file = [p{9} 'imu_raw.log'];
[acrud,wcrud,mcrud,tcrud,bcrud,~,~,T]=mong_read(imu_file,0,1);

avg = 1;
startup_runs = 800;
imu_calib = 512;
kalman_startup = 200*(~ctrl_ramp);

% Fake T
if(use_fake_T)
  T = [1:length(T)]'*mean(diff(T));
end

%% Load GPS data
if(use_gps)
  if(use_fake_gps)
    % Fake input, clear relevant data
    easting   = 0; westing = -easting;
    northing  = 0;
    elevation = 0;
    vx_gps    = 0;
    vy_gps    = 0;
    vz_gps    = 0;
    T_gps     = 10e-3;
  else
    % gps_file = '~/Escritorio/car/01.log';
    % [easting, northing, elevation, utmzone, sat, lat, lon, dop] = ...
    %     gpxlogger_xml_handler(gps_file, 1);
    % save('kalman/gps','easting','northing','elevation','utmzone','sat','lat','lon','dop');
    GPS       = load(gps_file);
    T_gps     = GPS(:,1);
    easting   = GPS(:,4); westing = -easting;
    northing  = GPS(:,5);
    elevation = GPS(:,6);
    vx_gps    = GPS(:,7);
    vy_gps    = GPS(:,8);
    vz_gps    = GPS(:,9);
  end
end

%% Re-calibrate sensors
% startup_runs samples are discarded
if (stabilize_ts)
  ts_ok_count = 0;
  i = 1;
  dT = diff(T);
  while(ts_ok_count < 10)
    if((dT(i) > 8000e-6) && (dT(i) < 12000e-6))
      ts_ok_count = ts_ok_count + 1;
      T_ok = i;
    else
      ts_ok_count = 0;
    end
    i = i + 1;
  end
  fprintf('Discarded %d samples during stabilization\n',T_ok);
  acrud = acrud(T_ok:end,:);
  wcrud = wcrud(T_ok:end,:);
  mcrud = mcrud(T_ok:end,:);
  bcrud = bcrud(T_ok:end,:);
  tcrud = tcrud(T_ok:end,:);
  T = T(T_ok(end):end);
else
  acrud = acrud(startup_runs:end,:);
  wcrud = wcrud(startup_runs:end,:);
  mcrud = mcrud(startup_runs:end,:);
  bcrud = bcrud(startup_runs:end,:);
  tcrud = tcrud(startup_runs:end,:);
  T = T(startup_runs:end);
end

% p0 and theta0 is estimated form first imu_calib samples
[a_calib,w_calib,euler_calib] = mong_conv(acrud(1:imu_calib,:),wcrud(1:imu_calib,:),mcrud(1:imu_calib,:),0,tcrud(1:imu_calib));
psi0   = mean(euler_calib(:,1));
phi0   = mean(euler_calib(:,2));
theta0 = mean(euler_calib(:,3));
b0     = mean(bcrud(1:imu_calib));

% averages are used
acrud(:,1) = moving_avg(acrud(:,1),avg); acrud(:,2) = moving_avg(acrud(:,2),avg); acrud(:,3) = moving_avg(acrud(:,3),avg);
wcrud(:,1) = moving_avg(wcrud(:,1),avg); wcrud(:,2) = moving_avg(wcrud(:,2),avg); wcrud(:,3) = moving_avg(wcrud(:,3),avg);
mcrud(:,1) = moving_avg(mcrud(:,1),avg); mcrud(:,2) = moving_avg(mcrud(:,2),avg); mcrud(:,3) = moving_avg(mcrud(:,3),avg);
bcrud      = moving_avg(bcrud,avg);
tcrud      = moving_avg(tcrud,avg);
% first imu_calib values are not used for kalman/control/etc
acrud = acrud(imu_calib+1:end,:);
wcrud = wcrud(imu_calib+1:end,:);
mcrud = mcrud(imu_calib+1:end,:);
bcrud = bcrud(imu_calib+1:end,:);
tcrud = tcrud(imu_calib+1:end,:);
T     = T(imu_calib+1:end,:);

[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0,tcrud);
b=altitud(bcrud,b0);

% gyro offset comp
w_calib_mean = mean(w_calib);
for i = 1:3
  w(:,i) = w(:,i) - w_calib_mean(i);
end

%% Constantes, entradas, observaciones e inicialización

% Constantes
N       = size(a,1);                   % Quantity of observation samples
Ns      = 15;                          % N states: cantidad de variables de estado de Kalman
Ngps    = 6;                           % N gps: cantidad de variables corregidas por gps
masa    = 1.741;                       % Quadcopter weight
w_hover = calc_omega(9.81*masa/4);     % At this velocity, motor's force equals weight
w_max   = 387;                         % Definition
w_min   = w_hover - (w_max - w_hover); % Only for simetry

%                  x   y   z  psi phi the vqx vqy vqz wqx wqy wqz ax  ay  az
Q_imu_gps = diag([1e2 1e2 1e2 1e0 1e0 1e0 1e2 1e2 1e2 1e1 1e1 1e1 1e0 1e0 1e0 ]);
%                 psi phi the ax  ay  az  wqx wqy wqz  x   y   z
R_imu_gps = diag([1e3 1e3 1e3 1e4 1e4 1e4 1e2 1e2 1e2 1e2 1e2 1e2]);
%                  x   y   z  psi phi the vqx vqy vqz wqx wqy wqz ax  ay  az
Q_imu     = diag([1e2 1e2 1e2 1e2 1e2 1e0 1e2 1e2 1e2 1e1 1e1 1e1 1e0 1e0 1e0]);
%                 psi phi the ax  ay  az  wqx wqy wqz  z
R_imu     = diag([1e1 1e1 1e3 1e4 1e4 1e4 1e2 1e2 1e2 1e3]);
%                  x   y   z  vqx vqy vqz
Q_gps     = diag([1e2 1e2 1e2 1e2 1e2 1e2]);
%                  x   y   z  vqx vqy vqz
R_gps     = diag([1e0 1e0 1e5 1e0 1e0 1e5]);

if(use_n_states == 0)
    K    = load('src/control/K.txt');
    sp_x = [0;0;0;theta0;0;0;0;0];
    Nctl = 8;
elseif(use_n_states == 1)
    Kp = load('src/control/K_prop.txt');
    Ki = load('src/control/K_int.txt');
    K = [Kp Ki];
    sp_x = [0;0;0;0;0;theta0;0;0;0;0];
    Nctl = 10;
    x_hat_integrals = zeros(1,2);
elseif(use_n_states == 2)
    K    = load('src/control/K_full.txt');
    sp_x = [0;0;0;0;0;theta0;0;0;0;0;0;0];
    Nctl = 12;
elseif(use_n_states == 3)
    fprintf('WARN: Matrix != main\n');
    Kp = load('src/control/K_prop_full.txt');
    Ki = load('src/control/K_int_full.txt');
    K = [Kp Ki];

    sp_x = [0;0;0;0;0;theta0;0;0;0;0;0;0;0;0;0;0];
    Nctl = 16;
    x_hat_integrals = zeros(1,4);
end
sp_w    = ones(4,1)*w_hover;

% Entradas
dw = zeros(N,4);        % Derivada de w. Cada columna corresponde a 1 motor
TM = drive(w);          % Fuerzas ejercidas por los motores en N. Cada columna corresponde a 1 motor.
D  = drag(w);           % Torque de Drag ejercido por los motores en N*m. Cada columna corresponde a cada motor
    
% Observaciones
z = [euler a w b];

% Inicialización
x_hat          = zeros(N,Ns);
P              = 1*eye(Ns);
w_control      = zeros(N-kalman_startup,4);
w_control(1,:) = w_hover*ones(size(w_control(1,:)));
x_hat_ctl      = zeros(N,Nctl);
P_gps          = 1*eye(Ngps);
gps_index      = 1;

x_hat(1,4:6) = [psi0, phi0, theta0];

%% Kalman

for i=2:N
    wc_i = i-kalman_startup;
    Dt = T(i) - T(i-1);
    Dt = min(Dt,12000e-6);Dt = max(Dt,000e-6); % Matches C 
    
    if ~allin1       
        % Kalman inercial
        if(i > kalman_startup + 1)
          % Use control output as current w
          [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,Q_imu,R_imu,Dt,...
              w_control(wc_i - 1,:)',z(i,:)', w_hover);
        else
          % Use set point w as current w
          [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,Q_imu,R_imu,Dt,...
              sp_w,z(i,:)', w_hover);
        end
        % Kalman GPS
        if(use_gps)
          if(T(i) >= T_gps(gps_index))
              [aux,P_gps]  = kalman_gps(x_hat(i-1,1:9),P_gps,Q_gps,R_gps,...
                  [northing(gps_index); westing(gps_index); elevation(gps_index); ...
                  vx_gps(gps_index); vy_gps(gps_index); vz_gps(gps_index)]);
              x_hat(i,1:3) = aux(1:3);
              if use_gps_vel
                  x_hat(i,7:9) = aux(4:6);
              end
            if(~use_fake_gps)
              gps_index = gps_index + 1;
            else
              % Fake gps is of size 1, so force gps_index==1 to always be true
              if(length(T) >= i + 100)
                % Call again after 100 samples at 10ms -> 1sec
                T_gps = T(i+100);
              else
                % No more GPS
                T_gps = inf;
              end
            end
          end
        end
    else % All in 1 big Kalman Filter
        if(i > kalman_startup + 1)
            % Use control output as current w
            if(use_gps)
                if(T(i) >= T_gps(gps_index))
                    % Sin velocidades del GPS
                    [x_hat(i,:),P] = kalman_imu_gps(x_hat(i-1,:),P,Q_imu_gps,...
                        R_imu_gps,Dt,w_control(wc_i - 1,:)',...
                        [z(i,1:end-1)';northing(gps_index);westing(gps_index);...
                        elevation(gps_index)], w_hover);
                    % Con velocidades del GPS
%                     [x_hat(i,:),P] = kalman_imu_gps(x_hat(i-1,:),P,Q_imu,...
%                         [R_imu(1:end-1,1:end-1) zeros(9,6);zeros(6,9) R_gps],Dt,...
%                         w_control(wc_i - 1,:)',[z(i,1:end-1)';northing(gps_index);...
%                         westing(gps_index); elevation(gps_index); vx_gps(gps_index);...
%                         vy_gps(gps_index); vz_gps(gps_index)], w_hover);
                    if(~use_fake_gps)
                        gps_index = gps_index + 1;
                    else
                        % Fake gps is of size 1, so force gps_index==1 to always be true
                        if(length(T) >= i + 100)
                            % Call again after 100 samples at 10ms -> 1sec
                            T_gps = T(i+100);
                        else
                            % No more GPS
                            T_gps = inf;
                        end
                    end
                else
                    [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,Q_imu,R_imu,Dt,...
                        w_control(wc_i - 1,:)',z(i,:)', w_hover);
                end
            end
        else
                % Use set point w as current w
                [x_hat(i,:),P] = kalman_imu(x_hat(i-1,:),P,Q_imu,R_imu,Dt,...
                    sp_w,z(i,:)', w_hover);
        end
    end
    
    % Control
    if(use_n_states == 0)
        x_hat_ctl(i,:) = [x_hat(i,3), x_hat(i,4), x_hat(i,5), x_hat(i,6), ...
            x_hat(i,9), x_hat(i,10), x_hat(i,11), x_hat(i,12)];
    elseif(use_n_states == 1)
        x_hat_integrals = x_hat_integrals + (Dt)*([x_hat(i,3) x_hat(i,6)] -sp_x(9:end)');
        x_hat_ctl(i,:) = [x_hat(i,3), x_hat(i,4), x_hat(i,5), x_hat(i,6), ...
            x_hat(i,9), x_hat(i,10), x_hat(i,11), x_hat(i,12) x_hat_integrals];
    elseif(use_n_states == 2)
        x_hat_ctl(i,:) = x_hat(i,1:12);
    elseif(use_n_states == 3)
        x_hat_integrals = x_hat_integrals + (Dt)*([x_hat(i,1:3) x_hat(i,6)] -sp_x(13:end)');
        x_hat_ctl(i,:)  = [x_hat(i,1:12) x_hat_integrals];
    end

    % First kalman_startup samples will not be used for control
    if~(i > kalman_startup + 1)
          continue;
    end
    w_control(wc_i,:) = (sp_w + K*(sp_x - x_hat_ctl(i,:)'))';
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
