global am at

close all
clear all
clc

wm=zeros(3,27);
wt=zeros(3,27);

k=0;
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
    for j=1:3
        if j==1
           vel='00';
        elseif j==2
            vel=%TODO vel1teo;
        elseif j==3;
            vel=%Todo vel2teo;
        
        end        
        for z=1:2
            if z==1 
                theta='00';
                k=k+1;
                 
            elseif z==2 && strcmp(vel,'00')
            else
                theta=%TODO theta teo;
                 k=k+1; 
            end
            str=['gyro/logs/' eje vel otro theta '.txt'];
            [a,w]=mong_read(str);
            wm(:,k)=[mean(w(:,1)); mean(w(:,2));mean(w(:,3))];
            wt(:,k)=w_teo(eje,vel,theta);  
        end
    end
end


%% Verificaciones

%Aceleraciones teóricas
at(:,1)=[];
at(:,13)=[];
at(:,21)=[];    
%Aceleraciones medidas
am(:,1)=[];
am(:,13)=[];
am(:,21)=[];    

%% La majuga

x0=[1/(3.9e-3*g) 1/(3.9e-3*g) 1/(3.9e-3*g) 0 0 0 0 0 0 0 0 0];

[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@cost,x0,[],[],optimset('MaxFunEvals',10000));

u=mean(RESIDUAL)
%sigma=sqrt(RESNORM/(length(a(1,:))-1));
sigma=std(RESIDUAL)

save X
