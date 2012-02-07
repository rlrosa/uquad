function [a,w] = imu_read(file)
%[a,w] = imu_read(file)
%Devuelve la aceleración en los tres ejes y la velocidad angular en los
%tres ejes
F  = fopen(file);
D  = textscan(F,'%s','delimiter','\t');
N  = length(D{1});
Nn = fix(N/9);
a=zeros(Nn,3);
w=zeros(Nn,3);
i = 1;

while  (9*(i-1)+8) < length(D{1})
    
%     a(i,1) = ( str2num(D{1}{9*(i-1)+3}) - offset ) * acc_sens;
%     a(i,2) = ( str2num(D{1}{9*(i-1)+4}) - offset ) * acc_sens;
%     a(i,3) = ( str2num(D{1}{9*(i-1)+5}) - offset ) * acc_sens;
%     w(i,1) = ( str2num(D{1}{9*(i-1)+6}) - offset ) * gyro_sens;
%     w(i,2) = ( str2num(D{1}{9*(i-1)+7}) - offset ) * gyro_sens;
%     w(i,3) = ( str2num(D{1}{9*(i-1)+8}) - offset ) * gyro_sens;   

    a(i,2) = str2num(D{1}{9*(i-1)+3});
    a(i,1) = str2num(D{1}{9*(i-1)+4});
    a(i,3) = str2num(D{1}{9*(i-1)+5});
    w(i,1) = str2num(D{1}{9*(i-1)+6});
    w(i,2) = str2num(D{1}{9*(i-1)+7});
    w(i,3) = str2num(D{1}{9*(i-1)+8});
    
    i=i+1;
    
end

%      figure()
%      subplot(211)
%      plot(a(:,1)); hold on; plot(a(:,2),'r'); plot(a(:,3),'g'); legend('a_x','a_y','a_z');
%      title('Aceleraciones lineales en cantidad de g')
%      subplot(212)
%      plot(w(:,1)); hold on; plot(w(:,2),'r'); plot(w(:,3),'g'); legend('w_x','w_y','w_z');
%      title('Velocidades angulares en °/s')
