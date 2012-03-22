close all
clear all
clc

fs = 50;

%% Cargo datos

% dir = 'subida';
dir = 'bajada';
% dir = 'parcial';
int = 2;

if strcmp(dir,'subida')
    if int==1
        file = 'subida_1-2012_02_23_xx_22_16_19';
        orden = 3;
    else
        file = 'subida_2-2012_02_23_xx_23_23_12';
        orden = 5;
    end
elseif strcmp(dir,'bajada')
	if int==1
        file = 'bajada_1-2012_02_23_xx_20_59_06';
        orden = 2;
    else
        file = 'bajada_2-2012_02_23_xx_22_53_30';
        orden = 4;
    end
else
    file = 'parcial-2012_02_23_xx_20_38_09';
    orden = 1;
end
        
[a,w,m,t_imu,b,fecha,ind]=mong_read...
    (['tests/mongoose/temperaturomometro/data/' file '.log'],0);
[aconv,wconv,mconv]=mong_conv(a,w,m,0);
t_imu=t_imu/10;

% a = a(1:200*50,:);
% w = w(1:200*50,:);
% m = m(1:200*50,:);
% 
% aconv = aconv(1:200*50,:);
% wconv = wconv(1:200*50,:);
% mconv = mconv(1:200*50,:);
% t_imu = t_imu(1:200*50);

%% Temperatura Posta
F = fopen('tests/mongoose/temperaturomometro/data/README');
D = textscan(F,'%s','delimiter','\t');
fclose(F);
k=1;
for i=1:length(D{1})
    if D{1}{i}(1)=='%'
        index(k)=i;
        k=k+1;
    end
end 

t_posta = str2double(D{1}(index(orden)+1:index(orden+1)-1));

%% Plot

[ax1,ax2] = plot_temp(fs,aconv,t_imu,t_posta,ind,1);

%% Ajuste de temperatura solamente en el eje z
    
% global a_crudas a_teoricos temperaturas
% 
% avr=10;
% a_crudas     = [mean(vec2mat(a(:,1),avr),2) mean(vec2mat(a(:,2),avr),2)...
%                 mean(vec2mat(a(:,3),avr),2)];
% N            = size(a_crudas,1);
% a_teoricos   = [zeros(N,1) zeros(N,1) -9.81*ones(N,1)];
% temperaturas = mean(vec2mat(t_imu,avr),2);
% 
% AJUSTE LINEAL
% x0_lin=0;
% [alpha_lin,RESNORM_lin,RESIDUAL_lin,EXITFLAG_lin]=lsqnonlin...
%     (@acc_temp_cost_lin,x0_lin,[],[],optimset('MaxFunEvals',10000));

%% Ajuste de temperatura todos los ejes
    
global a_crudas a_teoricos temperaturas to

avr=10;
a_crudas     = [mean(vec2mat(a(:,1),avr),2) mean(vec2mat(a(:,2),avr),2)...
                mean(vec2mat(a(:,3),avr),2)];
N            = size(a_crudas,1);
a_teoricos   = [zeros(N,1) zeros(N,1) -9.81*ones(N,1)];
temperaturas = mean(vec2mat(t_imu,avr),2);

% AJUSTE LINEAL
x0_lin=[.5 .5];
[param,RESNORM_lin,RESIDUAL_lin,EXITFLAG_lin]=lsqnonlin...
    (@temp_acc_cost,x0_lin,[],[],optimset('MaxFunEvals',10000));

%%

% %AJUSTE CUADRATICO
% x0=[0 0];
% [alpha,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin...
%     (@acc_temp_cost_cuad,x0,[],[],optimset('MaxFunEvals',10000));

%% Convierto

A    = load('acc','X','T_0');
% to   = A.T_0;
to = 20;

K=[A.X(1) 0 0;
    0 A.X(2) 0;
    0 0 A.X(3)];

% T=[1 -A.X(7) A.X(8);
%    A.X(9) 1 -A.X(10);
%    -A.X(11) A.X(12) 1];

b=[A.X(4) A.X(5) A.X(6)]';

% CON EL LINEAL
aconv_temp_lin=zeros(size(a));
for i=1:length(a(:,1))
    aux = (K*(1+param(1)*(t_imu(i)-to)))^-1 * (a(i,:)'-b*(1+param(2)*(t_imu(i)-to)));
%     aux=T*(((1+alpha_lin*(t_imu(i)-to))*K)^(-1))*(a(i,:)'-b);
    aconv_temp_lin(i,:)=aux';
end

break
%% Pruebas

k = 3;

figure 
    plot(t_imu,a,'+')
    hold on
    a_avg1=moving_avg(a(:,1),20);
    a_avg2=moving_avg(a(:,2),20);
    a_avg3=moving_avg(a(:,3),20);
    a_avg = [a_avg1 a_avg2 a_avg3];
    plot(t_imu,a_avg,'*')
    title('a contra temperatura')

%%


aconv_avg1=moving_avg(aconv(:,1),20);
aconv_avg2=moving_avg(aconv(:,2),20);
aconv_avg3=moving_avg(aconv(:,3),20);
aconv_avg = [aconv_avg1 aconv_avg2 aconv_avg3];

figure
    plot(t_imu,aconv,'+')
    hold on
    plot(t_imu,aconv_avg,'g*')
    title('aconv(z) contra temperatura')

    
    
%%
    
t_prueba = (min(t_imu):(max(t_imu)-min(t_imu))/(length(a(:,1))-1):max(t_imu));
Gox = A.X(1);
box = A.X(4);
Goy = A.X(2);
boy = A.X(5);
Goz = A.X(3);
boz = A.X(6);


alpha = .0025;

to = 20;
% to = A.T_0;

figure
    plot(t_imu,aconv_avg(:,3),'g*')
    hold on
%     plot(moving_avg(t_imu,20),moving_avg(((Gox*(1+alpha*(t_imu - to))).^-1.*(a(:,1)-box)),20),'*');
%     plot(moving_avg(t_imu,20),moving_avg(((Goy*(1+alpha*(t_imu - to))).^-1.*(a(:,2)-boy)),20),'*r');
    plot(moving_avg(t_imu,20),moving_avg(((Goz*(1+alpha*(t_imu - to))).^-1.*(a(:,3)-boz)),20),'*');
%     plot(t_prueba,Go*(1-alpha*(t_prueba' - to))+bo);

%%

for alpha = .001:.001:.009
    plot(t_imu,moving_avg(((Goz*(1+alpha*(t_imu - to))).^-1.*(a(:,3)-boz)),20),'*');
end


%%



























break

% CON EL LINEAL
aconv_temp_lin=zeros(size(a));
for i=1:length(a(:,1))
    aux=T*((K+[0 0 0;0 0 0;0 0 A.X(3)*alpha_lin*(t_imu(i)-to)])^(-1))*(a(i,:)'-b);
    aconv_temp_lin(i,:)=aux';
end


% CON EL CUADRATICO
aconv_temp=zeros(size(a));
for i=1:length(a(:,1))
    aux=T*(((1+alpha(1)*(t_imu(i)-to).^2+alpha(2)*(t_imu(i)-to))*K)^(-1))*(a(i,:)'-b);
    aconv_temp(i,:)=aux';
end

%%

t=0:1/fs:1/fs*(length(a(:,1))-1);
figure()
    plot(t,aconv_temp_lin(:,3),'g'); hold on; grid;
    plot(t,aconv(:,3),'r');
figure()
    plot(t_imu,aconv_temp_lin(:,3),'g'); hold on; grid;
    plot(t_imu,aconv(:,3),'r');
    
fprintf('Raiz de suma de errores al cuadrado\nK constante:%f\nK lineal:%f\n'...
    ,sqrt(sum((aconv(:,3)+9.81).^2)),sqrt(sum((aconv_temp_lin(:,3)+9.81).^2)))


figure()    
    plot(t,aconv_temp_lin(:,1)); hold on; 
    plot(t,aconv_temp_lin(:,2),'r'); plot(t,aconv_temp_lin(:,3),'g'); 
    grid; hold on; title('Compensacion lioneal y cuadratica de temperatura');
    plot(t,aconv_temp(:,1),'g'); hold on; 
    plot(t,aconv_temp(:,2),'b'); plot(t,aconv_temp(:,3),'r'); 
    
figure()
    plot(abs(aconv_temp_lin(:,3)+9.81))
    hold on; grid; title('errores')
    plot(abs(aconv(:,3)+9.81))
    plot(abs(aconv_temp(:,3)+9.81),'r')

figure()
    plot(t_imu,(aconv(:,3)+9.81)./t_imu)
    
figure()
    plot(te,A.X(1)*(1+alpha_lin*(te-to)))
    hold on
    plot(te,A.X(1)*ones(length(te),1),'g')
    plot(te,A.X(1)*(1+alpha(1)*(te-to).^2+alpha(2)*(te-to)),'r')
    title('Ganancia vs. Temperatura'); legend('Constante','Lineal','Cuadratica')

    
fprintf('Raiz de suma de errores al cuadrado\nK constante:%f\nK lineal:%f\nK cuadratico:%f\n'...
    ,sqrt(sum((aconv(:,3)+9.81).^2)),sqrt(sum((aconv_temp_lin(:,3)+9.81).^2)),...
    sqrt(sum((aconv_temp(:,3)+9.81).^2)))