function plot_main(x_hat, T_x_hat, z, T_z)

green1 = [154,205,50]/256;
green2 = [34,139,34]/256;
blue1  = [0,0,128]/256;
blue2  = [0,0,255]/256;
red1   = [255,0,0]/256;
red2   = [178,34,34]/256;

figure()
subplot(221)
    plot(T_z,z(:,end),'k*')
    hold on; grid
    plot(T_x_hat,x_hat(1:end,1),'color',blue2,'linewidth',2)
    plot(T_x_hat,x_hat(1:end,2),'color',red2,'linewidth',2)
    plot(T_x_hat,x_hat(1:end,3),'color',green2,'linewidth',2)
    handle = legend('\fontsize{15}Altura (barom)','\fontsize{15}x','\fontsize{15}y','\fontsize{15}z');
    set(handle, 'Box', 'off','location','northwest');
    title('\fontsize{15}Posicion [m]');
    xlabel('\fontsize{12}Tiempo desde ./main  [s]');
    ylabel('\fontsize{12}Posicion [m]');
    axis tight
    hold off    

subplot(222)
    plot(T_z,180/pi*z(:,1),'*-','color',blue1,'markersize',2)
    hold on; grid
    plot(T_z,180/pi*z(:,2),'*-','color',red1,'markersize',2)
    plot(T_z,180/pi*z(:,3),'*-','color',green1,'markersize',2)
    plot(T_x_hat,180/pi*x_hat(1:end,4),'color',blue2,'linewidth',2)
    plot(T_x_hat,180/pi*x_hat(1:end,5),'color',red2,'linewidth',2)
    plot(T_x_hat,180/pi*x_hat(1:end,6),'color',green2,'linewidth',2)
    handle = legend('\fontsize{15}\psi_{imu}','\fontsize{15}\phi_{imu}','\fontsize{15}\theta_{imu}','\fontsize{15}\psi_{est}','\fontsize{15}\phi_{est}','\fontsize{15}\theta_{est}');
    set(handle, 'Box', 'off','location','northwest');
    title('\fontsize{15}Orientacion [^o]')
    xlabel('\fontsize{12}Tiempo desde ./main  [s]');
    ylabel('\fontsize{12}Angulos de Euler [^o]');
    axis tight
    hold off    
    
subplot(223)
    plot(T_x_hat,x_hat(1:end,7),'color',blue2,'linewidth',2)
    hold on; grid
    plot(T_x_hat,x_hat(1:end,8),'color',red2,'linewidth',2)
    plot(T_x_hat,x_hat(1:end,9),'color',green2,'linewidth',2)
    handle = legend('\fontsize{15}v_{qx}','\fontsize{15}v_{qy}','\fontsize{15}v_{qz}');
    set(handle, 'Box', 'off','location','northwest');
    title('\fontsize{15}Velocidad [m/s]');
    xlabel('\fontsize{12}Tiempo desde ./main  [s]');
    ylabel('\fontsize{12}Velocidades lineales [m/s]');
    axis tight
    hold off

subplot(224)
    plot(T_z,z(:,7),'*-','color',blue1,'markersize',2)
    hold on; grid
    plot(T_z,z(:,8),'*-','color',red1,'markersize',2)
    plot(T_z,z(:,9),'*-','color',green1,'markersize',2)
    plot(T_x_hat,x_hat(1:end,10),'color',blue2,'linewidth',2)
    plot(T_x_hat,x_hat(1:end,11),'color',red2,'linewidth',2)
    plot(T_x_hat,x_hat(1:end,12),'color',green2,'linewidth',2)
    handle = legend('\fontsize{15}w_{qx}_{imu}','\fontsize{15}w_{qy}_{imu}','\fontsize{15}w_{qz}_{imu}','\fontsize{15}w_{qx}_{est}','\fontsize{15}w_{qy}_{est}','\fontsize{15}w_{qz}_{est}');
    set(handle, 'Box', 'off','location','northwest');
    title('\fontsize{15}\omega_q [rad/s]');
    xlabel('\fontsize{12}Tiempo desde ./main  [s]');
    ylabel('\fontsize{12}Velocidades angulares [rad/s]');
    axis tight
    hold off