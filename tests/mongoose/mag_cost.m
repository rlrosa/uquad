function f = mag_cost(x)

m1=evalin('base','mm');
m2=evalin('base','mt');

K=[x(1) 0 0;
    0 x(2) 0;
    0 0 x(3)];
T=[1 -x(7) x(8);
   x(9) 1 -x(10);
   -x(11) x(12) 1];


b=[x(4) x(5) x(6)]';
f=zeros(length(m1(1,:)),1);

for i=1:length(m1(1,:))
    f(3*i-2:3*i,1)=(m2(:,i)-T*(K^(-1))*(m1(:,i)-b));
    
end
    
