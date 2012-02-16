function w=w_teo(eje,vel,otro,theta)

syms ang1 ang2 ang3 Rx Ry Rz
t=str2num(theta)*pi/180;
p=0.5*pi/180;


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
    ang2=t;
    ang3=p;
    R1=eval(Ry);
    R2=eval(Rz);
    w = [-vel 0 0];
    
elseif eje=='y'
    
    
    ang3=t;
    ang1=p;	
    R1=eval(Rz);
    R2=eval(Rx);
    W = [0 vel 0];
    
elseif eje=='z'
    ang1=-t;
    ang2=p;
    R1=eval(Rx);
    R2=eval(Ry);
    G = [0 0 vel];
else
    fprintf('\nTe equivocaste vieja, tenés que pasar com eje "x", "y", o "z"\n')
end
R=R2*R1;
w=R*W';
