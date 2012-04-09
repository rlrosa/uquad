green1 = [154,205,50]/256;
green2 = [34,139,34]/256;
blue1  = [0,0,128]/256;
blue2  = [0,0,255]/256;
% red1   = [184,69,57]/256;
red1   = [165,136,105]/256;
red2   = [178,34,34]/256;

t      = cumsum(T);
width  = 3;
msize  = 4;

%% z00y45.txt

figure;
    plot(t,180/pi*z(:,1),'*-','color',blue1,'markersize',msize)
    hold on; grid
    plot(t,180/pi*z(:,2),'*-','color',red1,'markersize',msize)
    plot(t,180/pi*z(:,3),'*-','color',green1,'markersize',msize)
    plot(t,180/pi*x_hat(1:end,4),'color',blue2,'linewidth',width)
    plot(t,180/pi*x_hat(1:end,5),'color',red2,'linewidth',width)
    plot(t,180/pi*x_hat(1:end,6),'color',green2,'linewidth',width)
    handle = legend('\fontsize{15}\psi_{imu}','\fontsize{15}\phi_{imu}','\fontsize{15}\theta_{imu}','\fontsize{15}\psi_{est}','\fontsize{15}\phi_{est}','\fontsize{15}\theta_{est}');
    set(handle, 'Box', 'on','location','east');
    title('\fontsize{15}Orientacion [^o]')
    xlabel('\fontsize{12}Tiempo [s]');
    ylabel('\fontsize{12}Angulos de Euler [^o]');
    axis([t(1) t(end) -55 175])
    hold off    

%% z00y00.txt

figure;
    plot(t,z(:,end),'k*','markersize',msize)
    hold on; grid
    plot(t,x_hat(1:end,1),'color',blue2,'linewidth',width)
    plot(t,x_hat(1:end,2),'color',red2,'linewidth',width)
    plot(t,x_hat(1:end,3),'color',green2,'linewidth',width)
    handle = legend('\fontsize{15}Altura (barom)','\fontsize{15}x','\fontsize{15}y','\fontsize{15}z');
    set(handle, 'Box', 'off','location','southwest');
    title('\fontsize{15}Posicion [m]');
    xlabel('\fontsize{12}Tiempo [s]');
    ylabel('\fontsize{12}Posicion [m]');
    axis([t(1) t(end) -5 2])
    hold off    

figure;
    plot(t,180/pi*z(:,1),'*-','color',blue1,'markersize',msize)
    hold on; grid
    plot(t,180/pi*z(:,2),'*-','color',red1,'markersize',msize)
    plot(t,180/pi*z(:,3),'*-','color',green1,'markersize',msize)
    plot(t,180/pi*x_hat(1:end,4),'color',blue2,'linewidth',width)
    plot(t,180/pi*x_hat(1:end,5),'color',red2,'linewidth',width)
    plot(t,180/pi*x_hat(1:end,6),'color',green2,'linewidth',width)
    handle = legend('\fontsize{15}\psi_{imu}','\fontsize{15}\phi_{imu}','\fontsize{15}\theta_{imu}','\fontsize{15}\psi_{est}','\fontsize{15}\phi_{est}','\fontsize{15}\theta_{est}');
    set(handle, 'Box', 'off','location','east');
    title('\fontsize{15}Orientacion [^o]')
    xlabel('\fontsize{12}Tiempo [s]');
    ylabel('\fontsize{12}Angulos de Euler [^o]');
    axis([t(1) t(end) -10 180])
    hold off    
    
figure;
    plot(t,x_hat(1:end,7),'color',blue2,'linewidth',width)
    hold on; grid
    plot(t,x_hat(1:end,8),'color',red2,'linewidth',width)
    plot(t,x_hat(1:end,9),'color',green2,'linewidth',width)
    handle = legend('\fontsize{15}v_{qx}','\fontsize{15}v_{qy}','\fontsize{15}v_{qz}');
    set(handle, 'Box', 'off','location','northwest');
    title('\fontsize{15}Velocidad [m/s]');
    xlabel('\fontsize{12}Tiempo [s]');
    ylabel('\fontsize{12}Velocidades lineales [m/s]');
    axis tight
    hold off

figure;
subplot(311)
    hold on; grid;
    plot(t,z(:,7),'*-','color',blue1,'markersize',msize)
    plot(t,x_hat(1:end,10),'color',blue2,'linewidth',width)
    xlabel('\fontsize{12}Tiempo [s]');
    title('\fontsize{15}\omega_{qx} [rad/s]');
subplot(312)
    hold on; grid;
    plot(t,z(:,8),'*-','color',red1,'markersize',msize)
    plot(t,x_hat(1:end,11),'color',red2,'linewidth',width)
    xlabel('\fontsize{12}Tiempo [s]');
    title('\fontsize{15}\omega_{qy} [rad/s]');
subplot(313)
    hold on; grid;
    plot(t,z(:,9),'*-','color',green1,'markersize',msize)
    plot(t,x_hat(1:end,12),'color',green2,'linewidth',width)
    title('\fontsize{15}\omega_{qz} [rad/s]');  
    xlabel('\fontsize{12}Tiempo [s]');
    hold off