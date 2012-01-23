function [aconv,wconv] = imu_conv(a,w)

fs=100;
kax=6.4562;
kay=6.3798;
kaz=6.4719;
bax=507.3248;
bay=506.9135;
baz=507.9913;
ayza=0.0043;
azya=-0.0053;
axza=-0.0077;
azxa=0.0122;
axya= -0.0160;
ayxa= -0.0120;



Ka=[kax 0 0;
    0 kay 0;
    0 0 kaz];

ba=[bax; bay; baz];

Ta=[1 -ayza azya;
   axza 1 -azxa;
   -axya ayxa 1];
acorregida=zeros(size(a));

acorregida(:,1)=a(:,2);
acorregida(:,2)=a(:,1);
acorregida(:,3)=a(:,3);
a=acorregida;
aconv=zeros(size(a));
for i=1:length(a(:,1))
    aux=Ta*(Ka^(-1))*(a(i,:)'-ba);
    aconv(i,:)=aux';
end

wconv=w;



figure()
    subplot(211)
    plot(aconv(:,1)); hold on; plot(aconv(:,2),'r'); plot(aconv(:,3),'g'); legend('a_x','a_y','a_z');
    title('Aceleraciones lineales en m/(s^2)')
    subplot(212)
    plot(wconv(:,1)); hold on; plot(wconv(:,2),'r'); plot(wconv(:,3),'g'); legend('w_x','w_y','w_z');
    title('Velocidades angulares en °/s')
    
    
    %Desplazamiento
    T=1/fs;
    t=[0:T:T*(length(a(:,1))-1)];
    
    v=zeros(size(a));
    for i=1:length(v(:,1))-1
        v(i+1,:)=v(i,:)+aconv(i,:)*T;
    end
    
    x=zeros(size(a));
    
    for i=1:length(x(:,1))-1
        x(i+1,:)=x(i,:)+v(i,:)*T;
    end
    
    size(t)
    size(x)
    
    figure()
    subplot(211)
    plot(t,x(:,1),'b'); hold on ;plot(t,x(:,2),'r'); hold on ;plot(t,x(:,3),'g'); hold on  ;grid
    subplot(212)
    plot(t,v(:,1),'b'); hold on ;plot(t,v(:,2),'r'); hold on ;plot(t,v(:,3),'g'); hold on  ;grid
