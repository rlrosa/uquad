[a,w]=imu_read('./logs/4gdef/z10x45.txt');
[a,w]=imu_conv(a,w);
am=[mean(a(:,1));
    mean(a(:,2));
    mean(a(:,3))]

at=calcu_acc('z','10','45')


am-at


