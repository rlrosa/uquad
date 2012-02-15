function f=ang(x);


R1=[cos(x(1)) 0 -sin(x(1));
    0          1      0;
    sin(x(1))  0    cos(x(1))];


R2=[ cos(x(2)) sin(x(2)) 0;
    -sin(x(2)) cos(x(2)) 0;
    0 0 1];

R3=[ 1 0 0;
    0 cos(x(3)) sin(x(3)) ;
    0 -sin(x(3)) cos(x(3)) ];
R=R3*R2*R1;

a=evalin('base','aa');
f=R*[0 0 9.81]'-a;