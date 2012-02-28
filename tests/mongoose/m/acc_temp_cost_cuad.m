function f = acc_temp_cost(x)

crud = evalin('base','a_crudas');
teo  = evalin('base','a_teoricos');
t    = evalin('base','temperaturas');

A    = load('acc','X','T_0');
to   = A.T_0;

K=[A.X(1) 0 0;
    0 A.X(2) 0;
    0 0 A.X(3)];
T=[1 -A.X(7) A.X(8);
   A.X(9) 1 -A.X(10);
   -A.X(11) A.X(12) 1];

b=[A.X(4) A.X(5) A.X(6)]';

f=zeros(length(crud(:,1)),1);
for i=1:length(crud(:,1))
    f(1,3*i-2:3*i)=teo(i,:)'-T*(((1+x(1)*(t(i)-to).^2+x(2)*(t(i)-to))*K)^(-1))*(crud(i,:)'-b);    
end
    
