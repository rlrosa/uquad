function w=gyro_teo(eje,vel,bool_error_mecha,theta,bool_taladro)

vel=str2num(vel);
syms ang1 ang2 ang3 Rx Ry Rz
t=str2num(theta)*pi/180;
ang_error=0; % en º el error de meter la mecha en el cubo

Rx=[1 0 0;
    0 cos(ang1) sin(ang1);
    0 -sin(ang1) cos(ang1)];

Ry=[cos(ang2) 0 -sin(ang2);
    0          1      0;
    sin(ang2)  0    cos(ang2)];

Rz=[ cos(ang3) sin(ang3) 0;
    -sin(ang3) cos(ang3) 0;
    0 0 1];

if bool_taladro    
    if bool_error_mecha
        p=ang_error*pi/180;
    else
        p=0;
    end

    if eje=='x'
        ang2=t;
        ang3=p;
        R1=eval(Ry);
        R2=eval(Rz);
        W = [vel 0 0];
    elseif eje=='y'
        ang3=-t;
        ang1=p;	
        R1=eval(Rz);
        R2=eval(Rx);
        W = [0 -vel 0];
    elseif eje=='z'
        ang1=t;
        ang2=p;
        R1=eval(Rx);
        R2=eval(Ry);
        W = [0 0 -vel];
    end
else
    if eje=='x'
    ang3=-t;
    ang2=0;
    R1=eval(Ry);
    R2=eval(Rz);
    W = [vel 0 0];
elseif eje=='y'
    ang1=t;
    ang3=0;
    R1=eval(Rz);
    R2=eval(Rx);
    W = [0 -vel 0];
elseif eje=='z'
    ang2=t;
    ang1=0;
    R1=eval(Rx);
    R2=eval(Ry);
    W = [0 0 vel];
    end
end

R=R2*R1;
w=R*W';
