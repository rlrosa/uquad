function [aconv,wconv] = imu_conv(a,w)

fs=100;
kax=6.4932;
kay=6.3747;
kaz=6.3894;
bax=507.5670;
bay=506.9107;
baz=508.6760;
ayza=-1.6034e-04;
azya=-0.0021;
axza=-0.0054;
azxa=0.0130;
axya= -5.9798e-04;
ayxa= 0.0014;


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


  T=1/fs;
    t=[0:T:T*(length(a(:,1))-1)];
figure()
    %subplot(211)
    plot(t,aconv(:,1)); hold on; plot(t,aconv(:,2),'r'); plot(t,aconv(:,3),'g'); legend('a_x','a_y','a_z'); grid;
    title('Aceleraciones lineales en m/(s^2)')
    %subplot(212)
    %plot(wconv(:,1)); hold on; plot(wconv(:,2),'r'); plot(wconv(:,3),'g'); legend('w_x','w_y','w_z');
    %title('Velocidades angulares en °/s')
    
    
    %Desplazamiento
  
    
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
    %subplot(211)
    %plot(t,x(:,1),'b'); hold on ;
    plot(t,x(:,2),'r'); hold on ;
    plot(t,x(:,3),'g'); hold on  
    ;grid
    %subplot(212)
    %plot(t,v(:,1),'b'); hold on ;plot(t,v(:,2),'r'); hold on ;plot(t,v(:,3),'g'); hold on  ;grid
