% test_kalman_gps_subsystem: lo dice todo

gps_calib = 10;
gps       = load('kalman/gps.mat');
w         = -gps.easting+mean(gps.easting(1:gps_calib));
n         = gps.northing-mean(gps.northing(1:gps_calib));
e         = gps.elevation-mean(gps.elevation(1:gps_calib));

w=westing-mean(westing(1:gps_calib)); n=northing-mean(northing(1:gps_calib)); e=elevation;

N     = length(w);
Ns    = 9; % states
T     = 1; % supongo frecuencia de 1 Hz en los datos del gps
P     = eye(Ns);
Q     = diag(1e2*[1 1 1  1 1 1  1 1 1]);
R     = diag(1e5*[1 1 1]);
x_hat = zeros(N,Ns);

for i=2:N
    z              = [n(i);w(i);e(i)];
    [x_hat(i,:),P] = kalman_gps_subsystem(x_hat(i-1,:)',P,Q,R,T,z);
end

%% Plots

t      = T*(1:N);
green1 = [154,205,50]/256;
green2 = [34,139,34]/256;
blue1  = [0,0,128]/256;
blue2  = [0,0,255]/256;
red1   = [255,0,0]/256;
red2   = [178,34,34]/256;

figure;
subplot(311)
    hold on
    plot(t,n,'*','color',green1)
    plot(t,x_hat(:,1),'color',green2)
    title('\fontsize{16}x'); grid
    handle = legend('gps','estimacion'); set(handle, 'Box', 'off');
subplot(312)
    hold on
    plot(t,w,'*','color',red1)
    plot(t,x_hat(:,4),'color',red2)
    title('\fontsize{16}y'); grid
    handle = legend('gps','estimacion'); set(handle, 'Box', 'off');
subplot(313)
    hold on
    plot(t,e,'*','color',blue1)
    plot(t,x_hat(:,7),'color',blue2)
    title('\fontsize{16}z'); grid
    handle = legend('gps','estimacion'); set(handle, 'Box', 'off');
    
figure
    plot(x_hat(:,1),x_hat(:,4))