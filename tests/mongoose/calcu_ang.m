 function [theta,phi,psi]=calcu_ang(a,b)
%%--------------------------------------
% Devuelve los tres angulos de euler para a->b
% El orden de las rotaciones es: -theta según k
%                                -phi según j
%                                -psi según k
% ------------------------------------------

%El vector r es perpendicular a los otros dos. El ángulo se saca haciendo
%el producto escalar. 
r=vrrotvec(b/norm(b),a/norm(a));

%Rodrigues

I=diag([1 1 1]);
k=[r(1); r(2); r(3)];
kx=[0 -r(3) r(2);
    r(3) 0 -r(1);
    -r(2) r(1) 0];
R=I*cos(r(4))+kx*sin(r(4))+(1-cos(r(4)))*k*k';


%Verifico que no haya gimbal lock
%if (abs(R(1,3))~=1)Esta es la forma de hacerlo bien pero no anda siempre
%porque puede haber gimbal lock y por error en la medición no te das cuenta
if (abs(R(1,3))<0.99)
    phi=asin(-R(1,3))*180/pi;
    psi=atan2(R(2,3),R(3,3))*180/pi;
 
    theta=atan2(R(1,2),R(1,1))*180/pi;
else
    phi=180/pi*asin(R(1,3));
    psi=0;
    theta=180/pi*atan2(R(3,1),R(3,2));
end
    




