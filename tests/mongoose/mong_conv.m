function [aconv,wconv] = mong_conv(a,w)

fs=50;

%% Carga datos calibración
A=load('X');
kax=A.X(1);
kay=A.X(2);
kaz=A.X(3);
bax=A.X(4);
bay=A.X(5);
baz=A.X(6);
ayza=A.X(7);
azya=A.X(8);
axza=A.X(9);
azxa=A.X(10);
axya=A.X(11);
ayxa=A.X(12);

% % Parámetros usando todas las medidas
% kax=26.9278552355435;
% kay=27.1526603684984;
% kaz=25.9238055780634;
% bax=13.1925322021239;
% bay=-0.809714486048931;
% baz=-40.847571323247;
% ayza=-0.00929891109876383;
% azya=0.00931332704045358;
% axza=-6.56525688992829e-05;
% azxa=0.0056348798296733;
% axya= -0.00994182165668318;
% ayxa= -0.00381668872749988;

%% Conversion

Ka=[kax 0 0;
    0 kay 0;
    0 0 kaz];

ba=[bax; bay; baz];

Ta=[1 -ayza azya;
   axza 1 -azxa;
   -axya ayxa 1];

aconv=zeros(size(a));
for i=1:length(a(:,1))
    aux=Ta*(Ka^(-1))*(a(i,:)'-ba);
    aconv(i,:)=aux';
end

wconv=w;

T=1/fs;
t=0:T:T*(length(a(:,1))-1);
tp=0:10*T:T*(length(a(:,1))-1);
figure()
    %subplot(211)
    plot(t,aconv(:,1)); hold on; plot(t,aconv(:,2),'r'); plot(t,aconv(:,3),'g'); legend('a_x','a_y','a_z'); grid;
    title('Aceleraciones lineales en m/(s^2)')
    %subplot(212)
    %plot(wconv(:,1)); hold on; plot(wconv(:,2),'r'); plot(wconv(:,3),'g'); legend('w_x','w_y','w_z');
    %title('Velocidades angulares en °/s')


%Desplazamiento promediando 10 muestras
vp=zeros(floor(length(a(:,1))/10),3);
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
%     subplot(211)
    plot(t,x(:,1),'b'); hold on ;
    plot(t,x(:,2),'r'); hold on ;
    plot(t,x(:,3),'g'); hold on ; grid on
%     subplot(212)
%     plot(tp,xp(:,1),'b');hold on ;
%     plot(tp,xp(:,2),'r'); hold on ;
%     plot(tp,xp(:,3),'g'); hold on  ;grid
