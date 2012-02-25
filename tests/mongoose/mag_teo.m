function m=mag_teo(eje,theta,phi)

syms ang1 ang2 ang3 Rx Ry Rz

t=str2num(theta)*pi/180;
p=str2num(phi)*pi/180;

if str2num(phi)==18
    p=10*p;
end

x1=6039;
y1=5625;
x2=2118;
y2=6415;
giro=atan((y2-y1)/(x2-x1));

campo=[cos(giro) sin(giro) 0;
        -sin(giro) cos(giro) 0;
        0   0   1]*[  -0.0299496;0.1744918;.1477585];
   
    

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
    ang3=t;
    ang1=p;
    R1=eval(Rz);
    R2=eval(Rx);
    %C = [0.1477579 -0.0504510 0.1697052];
    C = [ campo(3) campo(1) campo(2) ];
    
elseif eje=='y'
    
    ang1=-t;
    ang2=p;
    R1=eval(Rx);
    R2=eval(Ry);
    %C = [-0.1697052 0.1477579 0.0504510];
    C = [ -campo(2) campo(3) -campo(1)];  
elseif eje=='z'
    ang2=t;
    ang3=-p;
    R1=eval(Ry);
    R2=eval(Rz);
    %C = [0.0504510 0.1697052 -0.1477579];
    C = [  -campo(1) campo(2) -campo(3)];  
else
    fprintf('\nTe equivocaste vieja, tenés que pasar com eje "x", "y", o "z"\n')
end
R=R2*R1;
m=R*C';