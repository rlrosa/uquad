function [aconv,wconv] = imu_conv(a,w)

fs=100;

%Calibración para 6g

% kax=6.4887;
% kay=6.3802;
% kaz=6.3898;
% bax=507.5408;
% bay=506.8803;
% baz=508.6485;
% ayza=5.9012e-04;
% azya=-0.0020;
% axza=-0.0046;
% azxa=0.0130;
% axya= -0.0013;
% ayxa= 6.0841e-04;

%Calibración 6g con las 12 primeras medidas
% kax=6.5002;
% kay=6.3956;
% kaz=6.3924;
% bax=507.6980;
% bay=506.7932;
% baz=508.6186;
% ayza=-0.0014;
% azya=0.0042;
% axza=-0.0040;
% azxa=0.0143;
% axya= -0.0020;
% ayxa= 2.4688e-4;

%Calibración 6g con las medidas 2:12
% kax=6.4883;
% kay=6.3786;
% kaz=6.3879;
% bax=507.5413;
% bay=506.8812;
% baz=508.6517;
% ayza=4.3935e-4;
% azya=0.0021;
% axza=-0.0045;
% azxa=0.0129;
% axya= -0.0017;
% ayxa= 0.0015;

% 
% % %Calibración 6g con torcidas
% kax=6.4926;
% kay=6.3883;
% kaz=6.3498;
% bax=507.5717;
% bay=506.8291;
% baz=508.8650;
% ayza=-0.0023;
% azya=0.0046;
% axza=-0.0036;
% azxa=0.0133;
% axya= 0.0032;
% ayxa= 0.0109;


%Calibración para 4g 
kax=9.7200;
kay=9.6034;
kaz=9.5701;
bax=507.6939;
bay=508.6372;
baz=506.1258;
ayza=-0.0051;
azya=0.0021;
axza=-0.0042;
azxa=0.0242;
axya= -0.0070;
ayxa= 0.0085;





Ka=[kax 0 0;
    0 kay 0;
    0 0 kaz];

ba=[bax; bay; baz];

Ta=[1 -ayza azya;
   axza 1 -azxa;
   -axya ayxa 1];
% acorregida=zeros(size(a));
% 
% acorregida(:,1)=a(:,2);
% acorregida(:,2)=a(:,1);
% acorregida(:,3)=a(:,3);
% a=acorregida;
aconv=zeros(size(a));
for i=1:length(a(:,1))
    aux=Ta*(Ka^(-1))*(a(i,:)'-ba);
    aconv(i,:)=aux';
end

wconv=w;


  T=1/fs;
    t=[0:T:T*(length(a(:,1))-1)];
    tp=[0:10*T:T*(length(a(:,1))-1)];
figure()
    %subplot(211)
    plot(t,aconv(:,1)); hold on; plot(t,aconv(:,2),'r'); plot(t,aconv(:,3),'g'); legend('a_x','a_y','a_z'); grid;
    title('Aceleraciones lineales en m/(s^2)')
    %subplot(212)
    %plot(wconv(:,1)); hold on; plot(wconv(:,2),'r'); plot(wconv(:,3),'g'); legend('w_x','w_y','w_z');
    %title('Velocidades angulares en °/s')
    
    
    %Desplazamiento promediando 10 muestras
    vp=zeros(length(a(:,1))/10,3);
    
     
    for i=1:length(vp(:,1))-1
        vp(i+1,:)=vp(i,:)+sum(aconv(i:i+9,:))*T;
        
    end
    xp=zeros(size(vp));
     for i=1:length(vp(:,1))-1
        xp(i+1,:)=xp(i,:)+vp(i,:)*10*T;
        
    end
    %Desplazamiento sin promediar
    v=zeros(size(a));
    for i=1:length(v(:,1))-1
        v(i+1,:)=v(i,:)+aconv(i,:)*T;
    end
    
    
    x=zeros(size(v));
  
    for i=1:length(x(:,1))-1
        x(i+1,:)=x(i,:)+v(i,:)*T;
        
    end
    
    
    
    
    figure()
    
    
    subplot(211)
    plot(t,x(:,1),'b'); hold on ;
    %plot(t,x(:,2),'r'); hold on ;
    plot(t,x(:,3),'g'); hold on  
    ;grid
    subplot(212)
%    plot(tp,xp(:,1),'b');hold on ;
    %plot(tp,xp(:,2),'r'); hold on ;
   %
%   plot(tp,xp(:,3),'g'); hold on  ;grid
