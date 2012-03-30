function plot_z(z)

figure()
subplot(221)
    title('kin - pos en m')
    plot(z(:,end),'k')
    legend('z')
    hold off    

subplot(222)
    title('kin - attitude en °')
    plot(180/pi*z(:,1:3))
    grid
    legend('\psi','\phi','\theta')
    hold off    
    
subplot(223)
    title('kin - vel en m/s')
    hold on; grid
%     legend('v_{qx}','v_{qy}','v_{qz}')
    hold off

subplot(224)
    title('kin - w en rad/s')
    plot(z(:,7:9))
    grid
    legend('w_x','w_y','w_z')
    hold off