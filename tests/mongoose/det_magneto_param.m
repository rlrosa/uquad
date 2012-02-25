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



%% La majuga

x0=[2048/1.1 2048/1.1 2048/1.1 0 0 0 0 0 0 0 0 0];

[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@mag_cost,x0,[],[],optimset('MaxFunEvals',10000,'MaxIter',1000));

u=mean(RESIDUAL)
sigma=std(RESIDUAL)

save('mag','X','sigma')

%% Otra calibración
Q0=[1 0 0    0 1 0    0 0 1];
b0=[0 0 0];
x0=[Q0 b0];
[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@mag_cost2,x0,[],[],optimset('MaxFunEvals',10000,'MaxIter',1000));

Q=[X(1) X(2) X(3);
    X(4) X(5) X(6);
    X(7) X(8) X(9)];
b=[X(10); X(11); X(12)];


Hm=0.2306004;
[V,D]=eig(Q*Q');
u=-2*Q'*b;
k=b'*Q*b-Hm^2;
a=4*Hm^2/(4*k-(V'*u)'*D^(-1)*(V'*u));

save('mag','V','D','b','a')

%% La posta?


%% La posta?

% U8 = [0.00473160006403247     -2.18916898319836e-05      0.000309423482503981;
%                          0       0.00455025410014059      7.83170752308679e-05;
%                          0                         0       0.00534686558080686];
%                      
% c8 =  [23.3152609806586;
%      -126.624459617958;
%       19.0429011162953];

U = [0.00473160006403247     -2.18916898319836e-05      0.000309423482503981;
                         0       0.00455025410014059      7.83170752308679e-05;
                         0                         0       0.00534686558080686];
                     
c=  [23.3152609806586;
    -126.624459617958;
    19.0429011162953];
  
  
for i=1:length(mm(1,:));
mc(:,i)=(U*(mm(:,i)-c));
end


