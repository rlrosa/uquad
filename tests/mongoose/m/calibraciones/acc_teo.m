function a=acc_teo(eje,theta,phi)

% Ajustado para funcionar con logs de acc/logs_marzo

syms ang1 ang2 ang3 Rx Ry Rz
t=str2num(theta)*pi/180;
p=str2num(phi)*pi/180;
g = 9.81;

Rx=[1 0 0;
    0 cos(ang1) sin(ang1);
    0 -sin(ang1) cos(ang1)];

Ry=[cos(ang2) 0 -sin(ang2);
    0          1      0;
    sin(ang2)  0    cos(ang2)];


Rz=[ cos(ang3) sin(ang3) 0;
    -sin(ang3) cos(ang3) 0;
    0 0 1];

if eje=='x'
    ang1=p;
    ang3=t;
    R1=eval(Rz);
    R2=eval(Rx);
    G = [g 0 0];
    
elseif eje=='y'
    ang1=-t;
    ang2=p;
    ang3=0;
    R1=eval(Rx);
    R2=eval(Ry);
    G = [0 g 0];

elseif eje=='z'
    
    ang1=0;
    ang2=t;
    ang3=-p;
    R1=eval(Ry);
    R2=eval(Rz);
    G = [0 0 -g];
    
else
    fprintf('\nTe equivocaste vieja, tenés que pasar com eje "x", "y", o "z"\n')
end
R=R2*R1;
a=R*G';