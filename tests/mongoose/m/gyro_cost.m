function f = gyro_cost(x)

a1=evalin('base','wm');
a2=evalin('base','wt');

K=[x(1) 0 0;
    0 x(2) 0;
    0 0 x(3)];
T=[1 -x(7) x(8);
   x(9) 1 -x(10);
   -x(11) x(12) 1];


b=[x(4) x(5) x(6)]';
f=zeros(length(a1(1,:)),1);
for i=1:length(a1(1,:))
    f(3*i-2:3*i,1)=a2(:,i)-T*(K^(-1))*(a1(:,i)-b);
    
end
    
    