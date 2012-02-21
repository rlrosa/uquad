function [aconv,wconv] = mong_conv(a,w)

fs=50;

%% Carga datos calibración

A=load('acc','X');
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

G=load('gyro','X');
kgx=G.X(1);
kgy=G.X(2);
kgz=G.X(3);
bgx=G.X(4);
bgy=G.X(5);
bgz=G.X(6);
gayza=G.X(7);
gazya=G.X(8);
gaxza=G.X(9);
gazxa=G.X(10);
gaxya=G.X(11);
gayxa=G.X(12);

%% Conversion

% ACELEROMETRO
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

% GIROSCOPO
Kg=[kgx 0 0;
    0 kgy 0;
    0 0 kgz];

bg=[bgx; bgy; bgz];

Tg=[1    -gayza gazya;
    gaxza 1    -gazxa;
   -gaxya gayxa 1];

wconv=zeros(size(w));
for i=1:length(w(:,1))
    auxg=Tg*(Kg^(-1))*(w(i,:)'-bg);
    wconv(i,:)=auxg';
end

T=1/fs;
t=0:T:T*(length(a(:,1))-1);
tp=0:10*T:T*(length(a(:,1))-1);
figure()
    subplot(211)
    plot(t,aconv(:,1)); hold on; plot(t,aconv(:,2),'r'); plot(t,aconv(:,3),'g'); legend('a_x','a_y','a_z'); grid;
    title('Aceleraciones lineales en m/(s^2)')
    subplot(212)
    plot(t,wconv(:,1)); hold on; plot(t,wconv(:,2),'r'); plot(t,wconv(:,3),'g'); legend('w_x','w_y','w_z'); grid;
    title('Velocidades angulares en °/s')


% %% Desplazamientos
%     
%     
% % %Desplazamiento promediando 10 muestras
% vp=zeros(floor(length(a(:,1))/10),3);
% for i=1:length(vp(:,1))-1
%    vp(i+1,:)=vp(i,:)+sum(aconv(i:i+9,:))*T;        
% end
% 
% xp=zeros(size(vp));
% 
%  for i=1:length(vp(:,1))-1
%     xp(i+1,:)=xp(i,:)+vp(i,:)*10*T;
% 
%  end
%      
% %Desplazamiento sin promediar
% v=zeros(size(a));
% for i=1:length(v(:,1))-1
%     v(i+1,:)=v(i,:)+aconv(i,:)*T;
% end
% 
% x=zeros(size(v));
% 
% for i=1:length(x(:,1))-1
%     x(i+1,:)=x(i,:)+v(i,:)*T;
% end
% 
% figure()
% %     subplot(211)
%     plot(t,x(:,1),'b'); hold on ;
%     plot(t,x(:,2),'r'); hold on ;
%     plot(t,x(:,3),'g'); hold on ; grid on
% %     subplot(212)
% %     plot(tp,xp(:,1),'b');hold on ;
% %     plot(tp,xp(:,2),'r'); hold on ;
% %     plot(tp,xp(:,3),'g'); hold on  ;grid
