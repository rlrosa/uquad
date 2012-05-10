% Correr un plot_c antes y esto anda todo

acc_orig = imu_data(:,4:6);

a=[0.4 0.3 0.15 0.15];
b=[0.2 0.2 0.2 0.2 0.1 0.1];

% for i=length(a):length(acc_orig(:,1))
%     conv_acc_orig_1_a_mano(i) = a*acc_orig(i:-1:i-length(a)+1,1);
% end
% figure; plot(conv_acc_orig_1_a_mano);hold on; plot(conv_acc_orig_1,'r')

conv_acc_orig_1=conv(acc_orig(:,1),a);
conv_acc_orig_2=conv(acc_orig(:,2),a);
conv_acc_orig_3=conv(acc_orig(:,3),a);
conv_acc_orig_a=[conv_acc_orig_1 conv_acc_orig_2 conv_acc_orig_3];

conv_acc_orig_1b=conv(acc_orig(:,1),b);
conv_acc_orig_2b=conv(acc_orig(:,2),b);
conv_acc_orig_3b=conv(acc_orig(:,3),b);
conv_acc_orig_b=[conv_acc_orig_1b conv_acc_orig_2b conv_acc_orig_3b];

alpha = 0.7;
for i=2:length(acc_orig(:,2))
    promedioy(i)=alpha*conv_acc_orig_b(i,2)+(1-alpha)*conv_acc_orig_b(i-1,2);
end    

figure;plot(acc_orig(:,2),'r');hold on;plot(promedioy);legend('a_y','filtrado 2 veces')

%%

wint7=(cumsum(imu_data(1500:end,7))*10e-3);

figure;
    hold on
    plot(imu_data(1500:end,10),'r')
    plot(wint7,'b')
    legend('psi sacado con acc','integral del gyro')
    title('\fontsize{16}x')
    axis tight