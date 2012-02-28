clear all
clc


mm=zeros(3,18);
mt=zeros(3,18);
am=zeros(3,18);
at=zeros(3,18);


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
            [a,w,c,b,t]=mong_read(str,0);
            mm(:,k)=[mean(c(:,1)); mean(c(:,2));mean(c(:,3))];
            [aconv,wconv,mconv]=mong_conv(a,w,c,0);
            am(:,k) =[mean(aconv(:,1)); mean(aconv(:,2));mean(aconv(:,3))];
            mt(:,k) =mag_teo(eje,'00',theta);  
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



%% La majuga

x0=[2048/1.1 2048/1.1 2048/1.1 0 0 0 0 0 0 0 0 0];

[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@mag_cost,x0,[],[],optimset('MaxFunEvals',10000,'MaxIter',1000));

u=mean(RESIDUAL);
sigma=std(RESIDUAL);

save('mag','X','sigma')



%% Resultado calibración bola y conversión

K = [0.00473160006403247     -2.18916898319836e-05      0.000309423482503981;
                         0       0.00455025410014059      7.83170752308679e-05;
                         0                         0       0.00534686558080686];
                     
b=  [23.3152609806586;
    -126.624459617958;
    19.0429011162953];


for i=1:length(mm(1,:));
mc(:,i)=(K*(mm(:,i)-b));
end

%% Convertir con nuestra calibración
M=load('mag','X');
kmx=M.X(1);
kmy=M.X(2);
kmz=M.X(3);
bmx=M.X(4);
bmy=M.X(5);
bmz=M.X(6);
mayza=M.X(7);
mazya=M.X(8);
maxza=M.X(9);
mazxa=M.X(10);
maxya=M.X(11);
mayxa=M.X(12);
%MAGNETOMETRO
Km=[kmx 0 0;        
    0 kmy 0;
    0 0 kmz];

bm=[bmx; bmy; bmz];

Tm=[1    -mayza mazya;
    maxza 1    -mazxa;
   -maxya mayxa 1];

mc=zeros(size(mm));
for i=1:length(mm(1,:))
    auxm=Tm*(Km^(-1))*(mm(:,i)-bm);
    mc(:,i)=auxm';
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


angulos=zeros(18,3);

for i=1:length(mc(1,:))
    [orientacion(i,1), orientacion(i,2), orientacion(i,3)] = calcu_ang(Norte,mt(:,i)); 
    [angulos(i,1),angulos(i,2),angulos(i,3)]=calcu_ang(Norte,mc(:,i));
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
for i=1:length(mc(1,:))
    [orientacion(i,2), orientacion(i,3)] = calcu_phi_psi(at(1,i), at(2,i), at(3,i)); 
    [phi,psi]=calcu_phi_psi(am(1,i), am(2,i), am(3,i));
    angulos(i,2)=phi;
    angulos(i,3)=psi;

    Rx=[1 0 0;
        0 cosd(psi) sind(psi);
        0 -sind(psi) cosd(psi)];

    Ry=[cosd(phi) 0 -sind(phi);
        0   1   0;
        sind(phi) 0 cosd(phi)];

    mrot=Ry^(-1)*Rx^(-1)*mc(:,i);

   
    angulos(i,1)=180/pi*atan2(mrot(1),mrot(2))+9.78;
    v=[-90+giro;-90+(giro+30);-90+(giro+45);-90+(giro+60);-90+(giro+90);-90+(giro+180);-90+giro;-90+(giro+30);-90+(giro+45);-90+(giro+60);-90+(giro+90);-90+(giro+180);180+giro;-180+giro+30;-180+giro+45;-180+giro+60;-180+giro+90;-180+giro+180];
    orientacion(:,1)=v;
    
end


error=angulos-orientacion;
error_theta=mean(error(:,1));
sigma_theta=std(error(:,1));