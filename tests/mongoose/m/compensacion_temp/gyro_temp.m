close all
clear all
clc

fs = 100;

%% Cargo datos

% dir = 'subida';
% % dir = 'bajada';
% % dir = 'parcial';
% int = 1;
% 
% if strcmp(dir,'subida')
%     if int==1
%         file = 'subida_1-2012_02_23_xx_22_16_19';
%         orden = 3;
%     else
%         file = 'subida_2-2012_02_23_xx_23_23_12';
%         orden = 5;
%     end
% elseif strcmp(dir,'bajada')
% 	if int==1
%         file = 'bajada_1-2012_02_23_xx_20_59_06';
%         orden = 2;
%     else
%         file = 'bajada_2-2012_02_23_xx_22_53_30';
%         orden = 4;
%     end
% else
%     file = 'parcial-2012_02_23_xx_20_38_09';
%     orden = 1;
% end
% 
% [a,w,m,t_imu,~,fecha,ind]=mong_read...
%     (['tests/mongoose/temperaturomometro/data/' file '.log'],0);
% [aconv,wconv,mconv]=mong_conv(a,w,m,0);
% t_imu=t_imu/10;
% 
% a = a(1:200*50,:);
% w = w(1:200*50,:);
% m = m(1:200*50,:);
% 
% aconv = aconv(1:200*50,:);
% wconv = wconv(1:200*50,:);
% mconv = mconv(1:200*50,:);
% t_imu = t_imu(1:200*50);



[a,w,m,t_imu,~,fecha,ind]=mong_read...
    (['tests/mongoose/temperaturomometro/data_abril/imu_raw.log'],0,1);
a = a(2.26e4:4.8e5,:);
w = w(2.26e4:4.8e5,:);
m = m(2.26e4:4.8e5,:);
t_imu=t_imu(2.26e4:4.8e5,:);

[aconv,wconv,mconv]=mong_conv(a,w,m,0);
t_imu=t_imu/10;



%% Temperatura Posta

% F = fopen('tests/mongoose/temperaturomometro/data/README');
% D = textscan(F,'%s','delimiter','\t');
% fclose(F);
% k=1;
% for i=1:length(D{1})
%     if D{1}{i}(1)=='%'
%         index(k)=i;
%         k=k+1;
%     end
% end 
% 
% t_posta = str2double(D{1}(index(orden)+1:index(orden+1)-1));

%% Plot

% [ax1,ax2] = plot_temp(fs,aconv,t_imu,t_posta,ind,1);
[ax1,ax2] = plot_temp(fs,wconv,t_imu,0,ind,1);

%% Ajuste
    
global w_crudas w_teoricos temperaturas to

A    = load('gyro','X','T_0');
to   = A.T_0;
% to = 20;

avr=10;

w_crudas     = [mean(vec2mat(w(:,1),avr),2) mean(vec2mat(w(:,2),avr),2)...
                mean(vec2mat(w(:,3),avr),2)];
N            = size(w_crudas,1);
w_teoricos   = [zeros(N,1) zeros(N,1) zeros(N,1)];
temperaturas = mean(vec2mat(t_imu,avr),2);

x0_lin=[0 0 0 0 0 0];
[x,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@temp_gyro_cost,x0_lin,[],[],optimset('MaxFunEvals',10000,'MaxIter',1000));

save('gyro_temp','x','to');

%% Convierto

K=[A.X(1) 0 0;
    0 A.X(2) 0;
    0 0 A.X(3)];

T=[1 -A.X(7) A.X(8);
   A.X(9) 1 -A.X(10);
   -A.X(11) A.X(12) 1];

b=[A.X(4) A.X(5) A.X(6)]';

wconv_temp_lin=zeros(size(w));  
for i=1:length(w(:,1))
    aux = T *(K^-1)* (w(i,:)'- (b + [x(1); ...
                                 x(2); ...
                                 x(3)]*(t_imu(i)-to)+[x(4);x(5);x(6)])) ;
    wconv_temp_lin(i,:)=aux';
end

%% Plots

w_avg1=moving_avg(w(:,1),20);
w_avg2=moving_avg(w(:,2),20);
w_avg3=moving_avg(w(:,3),20);
w_avg = [w_avg1 w_avg2 w_avg3];

wconv_avg1=moving_avg(wconv(:,1),20);
wconv_avg2=moving_avg(wconv(:,2),20);
wconv_avg3=moving_avg(wconv(:,3),20);
wconv_avg = [wconv_avg1 wconv_avg2 wconv_avg3];

figure
    plot(t_imu,wconv_avg(:,1),'g*')
    hold on
    plot(moving_avg(t_imu,20),moving_avg(wconv_temp_lin(:,1),20),'*c');
    plot(t_imu,wconv_avg(:,2),'r*')
    plot(moving_avg(t_imu,20),moving_avg(wconv_temp_lin(:,2),20),'*m');
    plot(t_imu,wconv_avg(:,3),'b*')
    plot(moving_avg(t_imu,20),moving_avg(wconv_temp_lin(:,3),20),'*y');
    axis tight; xlabel('Temperatura'); ylabel('Velocidad angular en rad/s');
    legend('wx sin compensar','wx compensada','wy sin compensar', ...
      'wy compensada', 'wz sin compensar','wz compensada')
        
figure()
subplot(211)
    plot(wconv_temp_lin(:,3),'g'); hold on; grid;
    plot(wconv(:,3),'r');
    axis([0 length(wconv_temp_lin) -10.5 -9.5])
subplot(212)
    plot(moving_avg(wconv_temp_lin(:,3),20),'g'); hold on; grid;
    plot(moving_avg(wconv(:,3),20),'r');
    axis([0 length(wconv_temp_lin) -10.5 -9.5])
    
%fprintf('Raiz de suma de errores al cuadrado\nK constante:%f\nK lineal:%f\n'...
 %   ,sqrt(sum((wconv(:,3)+9.81).^2)),sqrt(sum((wconv_temp_lin(:,3)+9.81).^2)))
    