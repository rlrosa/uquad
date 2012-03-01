clear all
clc

global mm mt

mm = zeros(18,3);
mt = zeros(18,3);
am = zeros(18,3);
at = zeros(3,18);
te = zeros(18,1);

k=1;
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
    for j=1:6
        if j==1
           theta='00';
        elseif j==2
            theta='30';
        elseif j==3;
            theta='45';
        elseif j==4
            theta='60';
        elseif j==5
            theta='90';
        else 
            theta='18';
        end        
            str=['./tests/mongoose/magnetometro/data_horizontal/' eje '00' otro theta ];
            [a,w,c,~,t]=mong_read(str,0);
            te(k,1) = mean(t);
            mm(k,:) =[mean(c(:,1)); mean(c(:,2));mean(c(:,3))];
            am(k,:) =[mean(a(:,1)); mean(a(:,2));mean(a(:,3))];
            mt(k,:) =mag_teo(eje,'00',theta);  
            if eje =='x'
                at(:,k)=[9.81;0;0];
            elseif eje =='y'
                at(:,k)=[0;9.81;0];
            else
                at(:,k)=[0;0;-9.81];
            end
                   
            k=k+1;
    end
end

T_0 = mean(te/10);

%% Calibracion mesa

x0=[2048/1.1 2048/1.1 2048/1.1 0 0 0 0 0 0 0 0 0];

[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@mag_cost,x0,[],[],optimset('MaxFunEvals',10000,'MaxIter',1000));

u=mean(RESIDUAL);
sigma=std(RESIDUAL);

save('mag','X','sigma','T_0')

[am,~,mc]=mong_conv(am,w,mm,0);

%% Calibración bola y conversión

K = [0.00473160006403247     -2.18916898319836e-05      0.000309423482503981;
                         0       0.00455025410014059      7.83170752308679e-05;
                         0                         0       0.00534686558080686];
                     
b=  [23.3152609806586;
    -126.624459617958;
    19.0429011162953];

for i=1:length(mm(1,:));
mc(i,:)=(K*(mm(i,:)'-b));
end

%% Diferencias de ángulos usando solo magneto

x1=6039;
y1=5625;
x2=2118;
y2=6415;
giro=atan((y2-y1)/(x2-x1));

orientacion=zeros(18,3);
dec = 99.78*pi/180;
inc = 39.85*pi/180;
Norte = [cos(inc)*cos(dec) cos(inc)*sin(dec) sin(inc)];
% Norte = [0.767724630032263 -0.631467617063739 0.108846410329734];
angulos=zeros(18,3);

for i=1:length(mc(:,1))
    [orientacion(i,1), orientacion(i,2), orientacion(i,3)] = calcu_ang(Norte,mt(i,:)); 
    [angulos(i,1),angulos(i,2),angulos(i,3)]=calcu_ang(Norte,mc(i,:));
end

error_theta = mean(orientacion(:,1)-angulos(:,1));
sigma_theta = std(orientacion(:,1)-angulos(:,1));

error_phi = mean(orientacion(:,2)-angulos(:,2));
sigma_phi = std(orientacion(:,2)-angulos(:,2));

error_psi = mean(orientacion(:,3)-angulos(:,3));
sigma_psi = std(orientacion(:,3)-angulos(:,3));

%% Diferencias de ángulos usando acc y magneto

orientacion=zeros(18,3);
angulos=zeros(18,3);
dec = 99.78*pi/180;
x1=6039;
y1=5625;
x2=2118;
y2=6415;
giro=180/pi*atan((y2-y1)/(x2-x1));
inc = 39.85*pi/180;
Norte = [cos(inc)*cos(dec) cos(inc)*sin(dec) sin(inc)];
for i=1:length(mc(:,1))
    [orientacion(i,2), orientacion(i,3)] = calcu_phi_psi(at(1,i), at(2,i), at(3,i)); 
    [phi,psi]=calcu_phi_psi(am(i,1), am(i,2), am(i,3));
    angulos(i,2)=phi;
    angulos(i,3)=psi;

    Rx=[1 0 0;
        0 cosd(psi) sind(psi);
        0 -sind(psi) cosd(psi)];

    Ry=[cosd(phi) 0 -sind(phi);
        0   1   0;
        sind(phi) 0 cosd(phi)];

    mrot=Ry^(-1)*Rx^(-1)*mc(i,:)';

   
    angulos(i,1)=180/pi*atan2(mrot(1),mrot(2))+9.78;
    v=[-90+giro;-90+(giro+30);-90+(giro+45);-90+(giro+60);-90+(giro+90);-90+(giro+180);-90+giro;-90+(giro+30);-90+(giro+45);-90+(giro+60);-90+(giro+90);-90+(giro+180);180+giro;-180+giro+30;-180+giro+45;-180+giro+60;-180+giro+90;-180+giro+180];
    orientacion(:,1)=v;
    
end


error=angulos-orientacion;
error_theta=mean(error(:,1));
sigma_theta=std(error(:,1));
