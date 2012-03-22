function f = temp_acc_cost_3(x)

crud = evalin('base','a_crudas');
teo  = evalin('base','a_teoricos');
t    = evalin('base','temperaturas');
to   = evalin('base','to');

A  = load('acc','X','T_0');

K=[A.X(1) 0      0;
    0     A.X(2) 0;
    0     0      A.X(3)];

% K=[x(1) 0    0;
%     0   x(2) 0;
%     0   0    x(3)];

% T=[1 -A.X(7) A.X(8);
%    A.X(9) 1 -A.X(10);
%    -A.X(11) A.X(12) 1];

b=[A.X(4) A.X(5) A.X(6)]';

% b=[x(4) x(5) x(6)]';

f=zeros(length(crud(:,1)),1);
for i=1:length(crud(:,1))
    f(3*i-2:3*i,1) = teo(i,:)'- ( ((K^-1) + [ x(1)*(t(i)-to) 0                0 ;              ...
                                           0            x(2)*(t(i)-to)   0 ;              ...
                                           0            0                x(3)*(t(i)-to) ] ...
                                  ) * (crud(i,:)'- (b + [x(4)*(t(i)-to); ...
                                                            x(5)*(t(i)-to); ...
                                                            x(6)*(t(i)-to)] ...
                                                       ) ) );
end