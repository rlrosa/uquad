function f = monod(x);
i2c=evalin('base','gi2c');
w=evalin('base','gw');
 
f= w-x(1)*i2c./(x(2)*i2c+x(3));
