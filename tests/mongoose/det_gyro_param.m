global am at

close all
clear all
% clc

wm=zeros(3,13);
wt=zeros(3,13);

k=1;
str=['gyro/logs/x00y00'];
[a,w]=mong_read(str);
wm(:,k)=[mean(w(:,1)); mean(w(:,2));mean(w(:,3))];
wt(:,k)=gyro_teo('x','00','y','00');  

for i=1:3
    if i==1
        eje='x';
        otro='y';
    elseif i==2
        eje='y';
        otro='z';
    else
        eje='z';
        otro='x';
    end
    for j=1:2
        if j==1
           vel='v1';
        elseif j==2
            vel='v2';
        end        
        for z=1:2
            if z==1
                theta='00';
                k=k+1;
            else
                theta='30';
                 k=k+1; 
            end
            str=['gyro/logs/' eje vel otro theta];
            [a,w]=mong_read(str,0);
            wm(:,k)=pi/180*[mean(w(:,1)); mean(w(:,2));mean(w(:,3))];
            if strcmp(vel,'v1');
                veloc=num2str(2*pi*2.2433);
            elseif strcmp(vel,'v2');
                veloc=num2str(2*pi*4.5383);
            else
                fprintf('\nBad name: det_gyro param - linea 49\n');
            end
            wt(:,k)=gyro_teo(eje,veloc,otro,theta);  
        end
    end
end


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

[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@gyro_cost,x0,[],[],optimset('MaxFunEvals',10000,'MaxIter',10000));

u=mean(RESIDUAL)
%sigma=sqrt(RESNORM/(length(a(1,:))-1));
sigma=std(RESIDUAL)

save X
