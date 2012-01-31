[a,w]=imu_read('./logs/medida2.txt');
[a,w]=imu_conv(a,w);
am=[mean(a(:,1));
    mean(a(:,2));
    mean(a(:,3))]


%Rotaciones para calibración
a=25.5;
b=37.2;
c=37.4;
%theta=asin((b-a)/c);

theta=22*pi/180;
phi = 45*pi/180;
%theta según y

R1=[cos(theta) 0 -sin(theta);
    0          1      0;
    sin(theta)  0    cos(theta)];


R2=[ cos(phi) sin(phi) 0;
    -sin(phi) cos(phi) 0;
    0 0 1];


R=R2*R1;

at=R*[0;0;9.81]

am-at


