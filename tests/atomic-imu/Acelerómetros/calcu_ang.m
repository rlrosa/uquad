%function [t,p,ps]=calcu_ang(a)
a=[-0.1479;
    -0.0880;
    9.9063];
syms theta phi psi
R1=[cos(theta) 0 -sin(theta);
    0          1      0;
    sin(theta)  0    cos(theta)];


R2=[ cos(phi) sin(phi) 0;
    -sin(phi) cos(phi) 0;
    0 0 1];

R3=[ 1 0 0;
    0 cos(psi) sin(psi) ;
    0 -sin(psi) cos(psi) ];

R=R3*R2*R1;
g=[0;0;9.81];

[t p ps]=solve(R*g-a);
    