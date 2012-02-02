function a=calcu_acc(eje,theta,phi)
 
syms ang1 ang2 ang3 Rx Ry Rz
t=str2num(theta)*pi/180;
p=str2num(phi)*pi/180;
sym I;
I=diag([1 1 1]);
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
    ang3=pi/2;
    ang1=pi/2+t;
    ang2=p;
    R1=eval(Rz);
    R2=eval(Rx);
    R3=eval(Ry);
    
    
elseif eje=='y'
    
    ang2=t;
    ang3=p;
    
    R1=eval(Ry);
    R2=eval(Rz);
    R3=I;

else
    ang1=-pi/2;
    ang3=t+pi/2;
    R1=eval(Rz*Rx);
    ang1=-p;
    R2=eval(Rx);
    
    R3=(I);
    
end
R=R3*R2*R1;
a=R*[0 0 9.81]';