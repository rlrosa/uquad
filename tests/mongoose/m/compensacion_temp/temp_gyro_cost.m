function f = temp_gyro_cost(x)

crud = evalin('base','w_crudas');
teo  = evalin('base','w_teoricos');
t    = evalin('base','temperaturas');
to   = evalin('base','to');

A  = load('gyro','X','T_0');

K=[A.X(1) 0      0;
    0     A.X(2) 0;
    0     0      A.X(3)];

T=[1 -A.X(7) A.X(8);
   A.X(9) 1 -A.X(10);
   -A.X(11) A.X(12) 1];

b=[A.X(4) A.X(5) A.X(6)]';

f=zeros(length(crud(:,1)),1);
for i=1:length(crud(:,1))
    f(3*i-2:3*i,1) = teo(i,:)'- ( T * (K^-1)*(crud(i,:)'- (b + [x(1); ...
                                x(2); ...
                                x(3)]*(t(i)-to)) ...
        ) );
end