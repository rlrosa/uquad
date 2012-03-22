function f = cost_bola(x)

mc = evalin('base','mc');

mag = load('mag.mat');
K = mag.K;
b = mag.b;

f=zeros(length(mc(:,1)),1);

for i=1:length(mc(:,1))
    f(i)=(mc(i,1)-x(1))^2+(mc(i,2)-x(2))^2+(mc(i,3)-x(3))^2-x(4)^2;
end
