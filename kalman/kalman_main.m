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

use_n_states = 3; % Regulates number of variables to control. Can be:
                    % 0: uses 8 states      -> [z psi phi tehta vqz wqx wqy wqz]
                    % 1: uses 8 states and their integrals
                    % 2: uses all 12 states -> [x y z psi phi tehta vqx vqy vqz wqx wqy wqz]
                    % 3: uses all 12 states and their integrals
use_gps      = 1; % Use kalman_gps
use_fake_gps = 0; % Feed kalman_gps with fake data (only if use_gps)

use_fake_T   = 0; % Ignore real timestamps from log, use average

allin1       = 1; % Includes inertial and gps Kalman all in 1 filter
use_gps_vel  = 0; % Uses velocity from GPS. Solo funca si allin1==0 (por ahora)

ctrl_ramp    = 1; % Run kalman+control when ramping motors.
stabilize_ts = 0; % Read out IMU data until stable Ts (matches main.c after 2012-04027).
                  % Should be set to 0 for logs dated 2012-05-13 and later.

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
if(stabilize_ts)
	warning('Should be set to 0 for logs dated 2012-05-13 and later!')
end

%% Source
log_path = 'tests/main/logs/2012_06_05_1_01_gps_afuera/';
if(~exist('log_path','var'))
	error('Must define a variable log_path to read from!');
end
imu_file  = [log_path '/imu_raw.log'];
gps_file  = [log_path '/gps.log'];

%% Load IMU data

% Imu
[acrud,wcrud,mcrud,tcrud,bcrud,~,~,T]=mong_read(imu_file,0,1);

avg = 1;
imu_calib = 512;
startup_samples = 100;

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
        northing  = GPS(:,4);
        westing = GPS(:,5);
		elevation = GPS(:,6);
		vx_gps    = GPS(:,7);
		vy_gps    = GPS(:,8);
		vz_gps    = GPS(:,9);
    end
    x0 = northing(1);
    y0 = westing(1);
else
    x0 = 0;
    y0 = 0;
end

% p0 and theta0 is estimated form first imu_calib samples
acrud_calib = mean(acrud(1:imu_calib,:));
wcrud_calib = mean(wcrud(1:imu_calib,:));
mcrud_calib = mean(mcrud(1:imu_calib,:));
tcrud_calib = mean(tcrud(1:imu_calib,:));
bcrud_calib = mean(bcrud(1:imu_calib,:));

[a_calib,w_calib,euler_calib] = mong_conv(acrud_calib,wcrud_calib,mcrud_calib,0,tcrud_calib);
acc0   = a_calib;
psi0   = euler_calib(:,1);
phi0   = euler_calib(:,2);
theta0 = euler_calib(:,3);
b0     = floor(mean(bcrud_calib));

% averages are used
acrud(:,1) = moving_avg(acrud(:,1),avg); acrud(:,2) = moving_avg(acrud(:,2),avg); acrud(:,3) = moving_avg(acrud(:,3),avg);
wcrud(:,1) = moving_avg(wcrud(:,1),avg); wcrud(:,2) = moving_avg(wcrud(:,2),avg); wcrud(:,3) = moving_avg(wcrud(:,3),avg);
mcrud(:,1) = moving_avg(mcrud(:,1),avg); mcrud(:,2) = moving_avg(mcrud(:,2),avg); mcrud(:,3) = moving_avg(mcrud(:,3),avg);
bcrud      = moving_avg(bcrud,avg);
tcrud      = moving_avg(tcrud,avg);

% low pass filter
h=[0.2 0.2 0.2 0.2 0.1 0.1];
hlen = length(h);
aux = conv(acrud(:,1),h); acrud(hlen:end,1) = aux(hlen:end-hlen+1);
aux = conv(acrud(:,2),h); acrud(hlen:end,2) = aux(hlen:end-hlen+1);
aux = conv(acrud(:,3),h); acrud(hlen:end,3) = aux(hlen:end-hlen+1);
aux = conv(wcrud(:,1),h); wcrud(hlen:end,1) = aux(hlen:end-hlen+1);
aux = conv(wcrud(:,2),h); wcrud(hlen:end,2) = aux(hlen:end-hlen+1);
aux = conv(wcrud(:,3),h); wcrud(hlen:end,3) = aux(hlen:end-hlen+1);
aux = conv(mcrud(:,1),h); mcrud(hlen:end,1) = aux(hlen:end-hlen+1);
aux = conv(mcrud(:,2),h); mcrud(hlen:end,2) = aux(hlen:end-hlen+1);
aux = conv(mcrud(:,3),h); mcrud(hlen:end,3) = aux(hlen:end-hlen+1);
aux = conv(bcrud ,h);      bcrud(hlen:end)   = aux(hlen:end-hlen+1);
aux = conv(tcrud,h);      tcrud(hlen:end)   = aux(hlen:end-hlen+1);

% first imu_calib values are not used for kalman/control/etc
% There's a couple of samples of offset with respect to C code, not sure
% why.
acrud = acrud(imu_calib+3:end,:);
wcrud = wcrud(imu_calib+3:end,:);
mcrud = mcrud(imu_calib+3:end,:);
bcrud = bcrud(imu_calib+3:end,:);
tcrud = tcrud(imu_calib+3:end,:);
T     = T(imu_calib+3:end,:);

[a,w,euler] = mong_conv(acrud,wcrud,mcrud,0,tcrud,T);
b=altitud(bcrud,b0);

% gyro offset comp
for i = 1:3
  w(:,i) = w(:,i) - w_calib(i);
end

%% Constantes, entradas, observaciones e inicialización

% Constantes
N         = size(a,1);                   % Quantity of observation samples
Ns        = 15;                          % N states: cantidad de variables de estado de Kalman
Ngps      = 6;                           % N gps: cantidad de variables corregidas por gps
masa      = 1.550;% fprintf('Ojo q la masa esta mal\n'); % Quadcopter weight
w_hover   = calc_omega(9.81*masa/4);     % At this velocity, motor's force equals weight
w_max     = 368;
w_min     = w_hover - (w_max - w_hover); % Only for simetry
DELTA_MAX = [1.0e-3 1.0e-3 0.5e-2 7.0e-3];
INT_MAX   = [1.0 1.0 2.0 2.0];

%                  x   y   z  psi  phi  thet vqx vqy vqz wqx wqy wqz ax  ay  az
Q_imu_gps = diag([1e2 1e2 1e-2 1e-2 1e-2 1e-5 1e2 1e2 1e2 1e-1 1e-1 1e-1 1e0 1e0 1e0 ]);
%                 psi phi the ax  ay  az  wqx wqy wqz  x   y   z
R_imu_gps = diag([1e2 1e2 1e5 1e4 1e4 1e4 1e-3 1e-3 1e-3 1e-2 1e-2 1e5]);
% R_imu_gps = diag([1e3 1e3 1e6 1e4 1e4 1e4 1e1  1e1  1e1  1e2 1e2 1e5]);
% %                  x   y   z  vqx vqy vqz
% Q_gps     = diag([1e2 1e2 1e2 1e2 1e2 1e2]);
% %                  x   y   z  vqx vqy vqz
% R_gps     = diag([1e0 1e0 1e5 1e0 1e0 1e5]);

Q_imu     = Q_imu_gps;
Rdiag     = diag(R_imu_gps);
R_imu     = diag([Rdiag(1:end-3); Rdiag(end)]);


if(use_n_states == 0)
    Kp   = load('src/control/K.txt');
    sp_x = [0;0;0;theta0;0;0;0;0];
    Nctl = 8;
elseif(use_n_states == 1)
    Kp = load('src/control/K_prop.txt');
    Ki = load('src/control/K_int.txt');
%     K = [Kp Ki];
    sp_x = [0;0;0;0;0;theta0;0;0;0;0];
    Nctl = 10;
    x_hat_integrals = zeros(1,2);
elseif(use_n_states == 2)
    Kp   = load('src/control/K_full.txt');
    sp_x = [x0;y0;0;0;0;theta0;0;0;0;0;0;0];
    Nctl = 12;
		x_hat_integrals = zeros(N,4);
elseif(use_n_states == 3)
%     fprintf('WARN: Matrix != main\n');
    Kp = load('src/control/K_prop_full_ppzt.txt');
    Ki = load('src/control/K_int_full_ppzt.txt');
%     K = [Kp Ki];

    sp_x = [x0;y0;1.5;0;0;theta0;0;0;0;0;0;0];
		x_hat_integrals = INT_MAX.*[1/-5.0 1/-6.6 0 0];
    Nctl = 16;
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
w_control      = zeros(N,4);
w_control(1,:) = w_hover*ones(size(w_control(1,:)));
w_c            = zeros(N,4);
w_c            = w_min*ones(size(w_control(1,:)));
x_hat_ctl      = zeros(N,Nctl);
P_gps          = 1*eye(Ngps);
gps_index      = 1;

x_hat(1,1:2) = [x0, y0];
x_hat(1,4:6) = [psi0, phi0, theta0];
x_hat(1,13:15) = acc0 - [0 0 9.81];

%% Kalman

for i=2:N
	Dt = T(i) - T(i-1);
	Dt = min(Dt,12000e-6);Dt = max(Dt,000e-6); % Matches C

	if ~allin1
		% Kalman inercial - Use control output as current w
		[x_hat(i,:),P,z(i,3)] = kalman_imu(x_hat(i-1,:),P,Q_imu,R_imu,Dt,...
			w_control(i - 1,:)',z(i,:)', w_hover);
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
		% Use control output as current w
		if(use_gps)
			if(T(i) >= T_gps(gps_index))
				% Sin velocidades del GPS
				[x_hat(i,:),P,z(i,3)] = kalman_imu_gps(x_hat(i-1,:),P,Q_imu_gps,...
					R_imu_gps,Dt,w_control(i - 1,:)',...
					[z(i,1:end-1)';northing(gps_index);westing(gps_index);...
					elevation(gps_index)], w_hover);
				% Con velocidades del GPS
				%                     [x_hat(i,:),P,z(i,3)] = kalman_imu_gps(x_hat(i-1,:),P,Q_imu,...
				%                         [R_imu(1:end-1,1:end-1) zeros(9,6);zeros(6,9) R_gps],Dt,...
				%                         w_control(i - 1,:)',[z(i,1:end-1)';northing(gps_index);...
				%                         westing(gps_index); elevation(gps_index); vx_gps(gps_index);...
				%                         vy_gps(gps_index); vz_gps(gps_index)], w_hover);
				if(~use_fake_gps)
					if gps_index+1 > length(T_gps)
                        T_gps(gps_index) = inf;
                    else
                        gps_index = gps_index + 1;
                    end
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
				[x_hat(i,:),P,z(i,3)] = kalman_imu(x_hat(i-1,:),P,Q_imu,R_imu,Dt,...
					w_control(i - 1,:)',z(i,:)', w_hover);
			end
		end
	end

	% Control
	if(use_n_states == 0)
		x_hat_ctl(i,:) = [x_hat(i,3), x_hat(i,4), x_hat(i,5), x_hat(i,6), ...
			x_hat(i,9), x_hat(i,10), x_hat(i,11), x_hat(i,12)];
		w_control(i,:) = (sp_w + Kp*(sp_x - x_hat(i,:)'))';
	elseif(use_n_states == 1)
		x_hat_integrals(i,:) = x_hat_integrals(i-1,:) + (Dt)*([sp_x(9:end)' - x_hat(i,3) x_hat(i,6)]);
		x_hat_ctl(i,:) = [x_hat(i,3), x_hat(i,4), x_hat(i,5), x_hat(i,6), ...
			x_hat(i,9), x_hat(i,10), x_hat(i,11), x_hat(i,12)];
		w_control(i,:) = (sp_w + Kp*(sp_x - x_hat_ctl(i,:)'))';
		w_control(i,:) = w_control(i,:) + Ki*x_hat_integrals(:,i);
	elseif(use_n_states == 2)
		x_hat_ctl(i,:) = x_hat(i,1:12);
		w_control(i,:) = (sp_w + Kp*(sp_x - x_hat_ctl(i,:)'))';
	elseif(use_n_states == 3)
		delta = (Dt)*([sp_x(4:5)' sp_x(3) sp_x(6)] - [x_hat(i,4:5) x_hat(i,3) x_hat(i,6)]);
		delta = sign(delta).*min(DELTA_MAX,abs(delta));
		x_hat_integrals(i,:) = x_hat_integrals(i-1,:) + delta;
		x_hat_integrals(i,:) = sign(x_hat_integrals(i,:)).*min(INT_MAX,abs(x_hat_integrals(i,:)));

		w_control(i,:) = (sp_w + Kp*(sp_x - x_hat(i,1:12)'))';
		w_control(i,:) = w_control(i,:) + (Ki*x_hat_integrals(i,:)')';

		%             x_hat_integrals(i,:) = x_hat_integrals(i-1,:) + (Dt)*([sp_x(4:5)' sp_x(3) sp_x(6)] - [x_hat(i,4:5) x_hat(i,3) x_hat(i,6)]);
		% 			x_hat_ctl(i,:)  = [x_hat(i,1:12) x_hat_integrals(i,:)];
		%             K=[Kp Ki];
		%             w_control(i,:) = (sp_w + K*([sp_x;0;0;0;0] - x_hat_ctl(i,:)'))';

	end
	for j=1:4
		if(w_control(i,j) < w_min)
			w_control(i,j) = w_min;
		end
		if (w_control(i,j) > w_max)
			w_control(i,j) = w_max;
		end
	end
	% Simulate C code ramp
	if~(i > startup_samples + 1)
		w_c(i,:) = w_control(i,:) - ...
			(startup_samples - i + 1)*(w_hover - w_min)/startup_samples;
	else
		w_c(i,:) = w_control(i,:);
	end

end

%% Plots

figure;
plot(T,180/pi*x_hat(:,6),'color',green1,'linewidth',2)
hold on
line([T(1) T(end)],[180/pi*theta0 180/pi*theta0],'color',blue1,'linewidth',2)
% title('\fontsize{16}\theta')
grid
xlabel('\fontsize{16}Tiempo (s)')
ylabel('\fontsize{16}\theta (grados)')
legend('\fontsize{18}\theta','\fontsize{18}\theta_0')

% figure; 
%     plot(w_control(:,1)+w_control(:,3)-w_control(:,2)-w_control(:,4),'r','linewidth',3); 
%     title('diferencia entre velocidades angulares (adelante+atras)-(derecha+izquierda)'); 
%     legend('Giro en z')

plot_main(x_hat,T,z,T);
plot_w([T w_c]);
