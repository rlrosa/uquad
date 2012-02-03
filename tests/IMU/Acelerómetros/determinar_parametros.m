global am at

g=9.81;
am=zeros(3,27);
at=zeros(3,27);
%Calibrar 4g
carpeta='4gdef';

%Calibrar 6g
%carpeta='6gdef';
k=0;
for i=1:3
    if i==1
        eje='x';
        otro='y';
    elseif i==2
            eje='y';
            otro='z';
    else eje='z';
        otro='x';
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
        else theta='45';
        end
        
        for z=1:2
            if z==1 
                phi='00';
                k=k+1;
                 
            elseif z==2 && strcmp(theta,'00')
            else phi='45';
                 k=k+1; 
            end
            str=['./logs/' carpeta '/' eje theta otro phi '.txt'];
            [a,w]=imu_read(str);
            am(:,k)=[mean(a(:,1)); mean(a(:,2));mean(a(:,3))];
            at(:,k)=calcu_acc(eje,theta,phi);  
        end
    end
end


% %plano sin inclinación
% %alfa6g
% alfa=0;
% %alfa4g
% %alfa=1.5*pi/180;
% at1=[0 g*cos(alfa) -g*sin(alfa)]';
% at2=[-g*sin(alfa) 0 g*cos(alfa)]';
% at3=[-g*cos(alfa) g*sin(alfa) 0]';
% 
% %Plano inclinado 20°
% a=36;
% b=23;
% c=37.4;
% %alfa6g
% alfa=asin((a-b)/c);
% 
% %alfapara4g
% %alfa=20*pi/180;
% at4=[0 g*cos(alfa) -g*sin(alfa)]';
% at5=[-g*sin(alfa) 0 g*cos(alfa)]';
% at6=[-g*cos(alfa) g*sin(alfa) 0]';
% 
% %Para el de 45
% a=35.8;
% b=9.7;
% c=37.4;
% %Alfa 6g
% alfa=asin((a-b)/c);
% 
% %alfa4g
% %alfa=45*pi/180;
% 
% at7=[0 g*cos(alfa) -g*sin(alfa)]';
% at8=[-g*sin(alfa) 0 g*cos(alfa)]';
% at9=[-g*cos(alfa) g*sin(alfa) 0]';
% 
% %para el de 60
% a=35.6;
% b=3.5;
% c=37.4;
% %ALFA6G
% alfa=asin((a-b)/c);
% %alfa4g
% %alfa=60*pi/180;
% 
% at10=[0 g*cos(alfa) -g*sin(alfa)]';
% at11=[-g*sin(alfa) 0 g*cos(alfa)]';
% at12=[-g*cos(alfa) g*sin(alfa) 0]';
% 
% %para el de 70
% a=35.6;
% b=0;
% c=37.4;
% %Alfa6g
% alfa=asin((a-b)/c);
% 
% %Alfa4g
% %alfa=70*pi/180;
% at13=[0 g*cos(alfa) -g*sin(alfa)]';
% at14=[-g*sin(alfa) 0 g*cos(alfa)]';
% at15=[-g*cos(alfa) g*sin(alfa) 0]';
% 
% 
% %Con dos giros
% %at16=calcu_acc(20,45);
% %at17=calcu_acc(45,45);
% %at18=calcu_acc(60,45);
% %at19=calcu_acc(70,45);
% %Aceleraciones medidas
% 
% [a,w]=imu_read('logs/6g/x_0_y.txt');
% am1=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/y_0_z.txt');
% am2=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/z_0_-x.txt');
% am3=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/x_20_y.txt');
% am4=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/y_20_z.txt');
% am5=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/z_20_-x.txt');
% am6=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/x_45_y.txt');
% am7=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/y_45_z.txt');
% am8=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/z_45_-x.txt');
% am9=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/x_60_y.txt');
% am10=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/y_60_z.txt');
% am11=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/z_60_-x.txt');
% am12=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/x_70_y.txt');
% am13=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/y_70_z.txt');
% am14=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% [a,w]=imu_read('logs/6g/z_70_-x.txt');
% am15=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% 
% %  [a,w]=imu_read('logs/6g/y_20_z_45.txt');
% %  am16=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% % 
% % [a,w]=imu_read('logs/6g/y_45_z_45.txt');
% % am17=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% % 
% % [a,w]=imu_read('logs/6g/y_60_z_45.txt');
% % am18=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
% % 
% % [a,w]=imu_read('logs/6g/y_70_z_45.txt');
% % am19=[mean(a(:,1)) mean(a(:,2)) mean(a(:,3))]';
%%
%Aceleraciones teóricas

at(:,1)=[];
at(:,13)=[];
at(:,21)=[];    
%Aceleraciones medidas
am(:,1)=[];
am(:,13)=[];
am(:,21)=[];    

%%
x0=[512/(6*g) 512/(6*g) 512/(6*g) 512 512 512 0 0 0 0 0 0];

LB=[5 5 5 500 500 500 -5 -5 -5 -5 -5 -5];
UP=[10 10 10 520 520 520 5 5 5 5 5 5];
[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@cost,x0,[],[],optimset('MaxFunEvals',10000));


u=mean(RESIDUAL)
%sigma=sqrt(RESNORM/(length(a(1,:))-1));
sigma=std(RESIDUAL)