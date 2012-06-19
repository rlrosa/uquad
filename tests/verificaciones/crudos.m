[a,w,euler] = mong_conv(imu_raw(:,4:6),imu_raw(:,7:9),imu_raw(:,10:12),0,imu_raw(:,13),t);

figure;
    plot(imu_raw(:,1),a(:,1),'color',blue1)
    hold on
    plot(imu_raw(:,1),a(:,2),'color',red1)
    plot(imu_raw(:,1),a(:,3),'color',green1)
    legend('ax','ay','az')

figure;
    plot(imu_raw(:,1),w(:,1),'color',blue1)
    hold on
    plot(imu_raw(:,1),w(:,2),'color',red1)
    plot(imu_raw(:,1),w(:,3),'color',green1)
    legend('wx','wy','wz')
    
figure;
    plot(imu_raw(:,1),180/pi*euler(:,1),'color',blue1)
    hold on
    plot(imu_raw(:,1),180/pi*euler(:,2),'color',red1)
    plot(imu_raw(:,1),180/pi*euler(:,3),'color',green1)
    legend('\psi','\phi','\theta')


% figure;
%     plot(imu_raw(:,1),imu_raw(:,4),'b')
%     hold on
%     plot(imu_raw(:,1),imu_raw(:,5),'r')
%     plot(imu_raw(:,1),imu_raw(:,6),'g')
%     legend('ax','ay','az')
% 
% figure;
%     plot(imu_raw(:,1),imu_raw(:,7),'b')
%     hold on
%     plot(imu_raw(:,1),imu_raw(:,8),'r')
%     plot(imu_raw(:,1),imu_raw(:,9),'g')
%     legend('wx','wy','wz')
%     
% figure;
%     plot(imu_raw(:,1),imu_raw(:,10),'b')
%     hold on
%     plot(imu_raw(:,1),imu_raw(:,11),'r')
%     plot(imu_raw(:,1),imu_raw(:,12),'g')
%     legend('mx','my','mz')