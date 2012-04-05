close all
clear all
% clc

global wm wt

[archivos,N]=cargo_nombres_gyro();
wm=zeros(3,N);
wt=zeros(3,N);
for i=1:N
    [a,w,c,t,b]=mong_read(archivos{i},0);
    wm(:,i)=[mean(w(:,1)); mean(w(:,2));mean(w(:,3))];
    if strcmp(archivos{i}(12:13),'v1');
        veloc=num2str(2*pi*2.2433);
    elseif strcmp(archivos{i}(12:13),'v2');            
        veloc=num2str(2*pi*4.5383);
    elseif strcmp(archivos{i}(12:13),'v3');            
        veloc=num2str(2*pi*0.556);
    elseif strcmp(archivos{i}(12:13),'00');            
        veloc=num2str(0);    
    else
        fprintf('\nBad name: det_gyro param - linea 22\n');
    end
    wt(:,i)=gyro_teo(archivos{i}(11),veloc,strcmp(archivos{i}(12:13),'v2'),archivos{i}(15:16),~strcmp(archivos{i}(12:13),'v3'));  
    te(i,1) = mean(t);
end
T_0 = mean(te/10);


%% Verificaciones

% %Aceleraciones teóricas
% at(:,1)=[];
% at(:,13)=[];
% at(:,21)=[];    
% %Aceleraciones medidas
% am(:,1)=[];
% am(:,13)=[];
% am(:,21)=[];    

%% La majuga

x0=[1 1 1 0 0 0 0 0 0 0 0 0];

[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@gyro_cost,x0,[],[],optimset('MaxFunEvals',100000,'MaxIter',100000));

u=mean(abs(RESIDUAL))
%sigma=sqrt(RESNORM/(length(a(1,:))-1));
sigma=std(RESIDUAL)

save('gyro','X','sigma','T_0');