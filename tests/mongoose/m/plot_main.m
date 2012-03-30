function plot_main(x_hat, z)

figure()
subplot(221)
    title('pos en m')
    plot([x_hat(1:end,1)],'b')
    hold on; grid
    plot([x_hat(1:end,2)],'r')
    plot(z(:,end),'k')
    plot([x_hat(1:end,3)],'g')
    legend('x','y','z')
    hold off    

subplot(222)
    title('attitude en °')
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
    title('vel en m/s')
    plot([x_hat(1:end,7)],'b')
    hold on; grid
    plot([x_hat(1:end,8)],'r')
    plot([x_hat(1:end,9)],'g')
    legend('v_{qx}','v_{qy}','v_{qz}')
    hold off

subplot(224)
    title('w en rad/s')
    plot(z(:,7),'k')
    hold on; grid
    plot(z(:,8),'k')
    plot(z(:,9),'k')    
    plot([x_hat(1:end,10)],'b')
    plot([x_hat(1:end,11)],'r')
    plot([x_hat(1:end,12)],'g')
    legend('w_x','w_y','w_z','w_x','w_y','w_z')
    hold off