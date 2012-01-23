global am at

g=9.81;
%Medidas de ángulos

%plano sin inclinación
at1=[0 g 0]';
at2=[0 0 g]';
at3=[-g 0 0]';

%Plano inclinado 20°
a=36;
b=23;
c=37.4;

alfa=asin((a-b)/c);

at4=[0 g*cos(alfa) -g*sin(alfa)]';
at5=[-g*sin(alfa) 0 g*cos(alfa)]';
at6=[-g*cos(alfa) g*sin(alfa) 0]';

%Para el de 45
a=35.8;
b=9.7;
c=37.4;

alfa=asin((a-b)/c);

at7=[0 g*cos(alfa) -g*sin(alfa)]';
at8=[-g*sin(alfa) 0 g*cos(alfa)]';
%at9=[-g*cos(alfa) g*sin(alfa) 0]';

%para el de 60
a=35.6;
b=3.5;
c=37.4;
alfa=asin((a-b)/c);


at10=[0 g*cos(alfa) -g*sin(alfa)]';
at11=[-g*sin(alfa) 0 g*cos(alfa)]';
at12=[-g*cos(alfa) g*sin(alfa) 0]';

%para el de 70
a=35.6;
b=0;
c=37.4;

alfa=asin((a-b)/c);

at13=[0 g*cos(alfa) -g*sin(alfa)]';
at14=[-g*sin(alfa) 0 g*cos(alfa)]';
at15=[-g*cos(alfa) g*sin(alfa) 0]';

%Aceleraciones teóricas



a=[at1 at2 at3 at4 at5 at6 at7 at8 at9 at10 at11 at12 at13 at14 at15];
assignin('base','at',a);

%Aceleraciones medidas

[a,w]=imu_read('logs/x_0_y.txt');
am1=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/y_0_z.txt');
am2=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/z_0_-x.txt');
am3=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/x_20_y.txt');
am4=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/y_20_z.txt');
am5=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/z_20_-x.txt');
am6=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/x_45_y.txt');
am7=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/y_45_z.txt');
am8=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/z_0_-x.txt');
%am9=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/x_60_y.txt');
am10=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/y_60_z.txt');
am11=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/z_60_-x.txt');
am12=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/x_70_y.txt');
am13=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/y_70_z.txt');
am14=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

[a,w]=imu_read('logs/z_70_-x.txt');
am15=[mean(a(:,2)) mean(a(:,1)) mean(a(:,3))]';

a=[am1 am2 am3 am4 am5 am6 am7 am8 am9 am10 am11 am12 am13 am14 am15];

assignin('base','am',a);



%%
x0=[512/(6*g) 512/(6*g) 512/(6*g) 512 512 512 0 0 0 0 0 0];

LB=[5 5 5 500 500 500 -5 -5 -5 -5 -5 -5];
UP=[10 10 10 520 520 520 5 5 5 5 5 5];
[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@cost,x0,[],[],optimset('MaxFunEvals',10000))
