global mm mt

close all
clear all
clc


mm=zeros(3,27);
mt=zeros(3,27);

k=0;
for i=1:3
    if i==1
        eje='x';
        otro='z';
    elseif i==2
        eje='y';
        otro='x';
    else
        eje='z';
        otro='y';
    end
    for j=1:5
        if j==1
           theta='00';
        elseif j==2
            theta='10';
        elseif j==3;
            theta='20';
        elseif j==4
            theta='30';
        else
            theta='45';
        end        
        for z=1:2
            if z==1 
                phi='00';
                k=k+1;
                 
            elseif z==2 && strcmp(theta,'00')
            else
                phi='45';
                 k=k+1; 
            end
            str=['./tests/mongoose/magnetometro/data/' eje theta otro phi ];
            [a,w,c,b,t]=mong_read(str,0);
            mm(:,k)=[mean(c(:,1)); mean(c(:,2));mean(c(:,3))];
            mt(:,k)=mag_teo(eje,theta,phi);  
        end
    end
end


%% Verificaciones

%Aceleraciones teóricas
%at(:,1)=[];
%at(:,13)=[];
%at(:,21)=[];    
%Aceleraciones medidas
%am(:,1)=[];
%am(:,13)=[];
%am(:,21)=[];    

%% La majuga

x0=[2048/.88 2048/.88 2048/.88 0 0 0 0 0 0 0 0 0];

[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@mag_cost,x0,[],[],optimset('MaxFunEvals',10000,'MaxIter',1000));

u=mean(RESIDUAL)
sigma=std(RESIDUAL)

save('mag','X','sigma')