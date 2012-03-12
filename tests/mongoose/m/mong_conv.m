
function [aconv,wconv,euler] = mong_conv(a,w,m,plotear)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [aconv,wconv,mconv] = mong_conv(a,w,m,plotear)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if(nargin < 4)
  plotear = 1;
end

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

M=load('mag','X');
kmx=M.X(1);
kmy=M.X(2);
kmz=M.X(3);
bmx=M.X(4);
bmy=M.X(5);
bmz=M.X(6);
mayza=M.X(7);
mazya=M.X(8);
maxza=M.X(9);
mazxa=M.X(10);
maxya=M.X(11);
mayxa=M.X(12);
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


% MAGNETOMETRO


K = [0.00473160006403247     -2.18916898319836e-05      0.000309423482503981;
                         0       0.00455025410014059      7.83170752308679e-05;
                         0                         0       0.00534686558080686];
                     
b=  [23.3152609806586;
    -126.624459617958;
    19.0429011162953];

% Km=[kmx 0 0;        
%     0 kmy 0;
%     0 0 kmz];
% 
% bm=[bmx; bmy; bmz];
% 
% Tm=[1    -mayza mazya;
%     maxza 1    -mazxa;
%    -maxya mayxa 1];

mconv = zeros(size(m));
euler = zeros(size(m));
for i=1:length(m(:,1))
%     auxm=Tm*(Km^(-1))*(m(i,:)'-bm);
    auxm = K*(m(i,:)'-b);
    mconv(i,:)=auxm';
    
    % Convierto a angulos de Euler
    if (abs(aconv(i,1))<9.72)
        phi=-180/pi*asin(aconv(i,1)/9.81);
        psi=180/pi*atan2(aconv(i,2),aconv(i,3));
    elseif aconv(i,1) > 0
        phi=-90;
        psi=0;
    else 
        phi=90;
        psi=0;
    end
    mrot = [ cosd(phi)/(cosd(phi)^2 + sind(phi)^2), (sind(phi)*sind(psi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2)), (cosd(psi)*sind(phi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2));
                                                 0,                                           cosd(psi)/(cosd(psi)^2 + sind(psi)^2),                                         -sind(psi)/(cosd(psi)^2 + sind(psi)^2);
            -sind(phi)/(cosd(phi)^2 + sind(phi)^2), (cosd(phi)*sind(psi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2)), (cosd(phi)*cosd(psi))/((cosd(phi)^2 + sind(phi)^2)*(cosd(psi)^2 + sind(psi)^2))]...
            *auxm;
    theta=180/pi*atan2(mrot(1),mrot(2))+9.78;
    euler(i,:)=[psi, phi, theta];
end

if plotear
    T=1/fs;
    t=0:T:T*(length(a(:,1))-1);
    tp=0:10*T:T*(length(a(:,1))-1);
    figure()
        subplot(311)
        plot(t,aconv(:,1)); hold on; plot(t,aconv(:,2),'r'); plot(t,aconv(:,3),'g'); legend('a_x','a_y','a_z'); grid;
        title('Aceleraciones lineales en m/(s^2)')
        subplot(312)
        plot(t,wconv(:,1)); hold on; plot(t,wconv(:,2),'r'); plot(t,wconv(:,3),'g'); legend('w_x','w_y','w_z'); grid;
        title('Velocidades angulares en °/s')
        subplot(313)
        plot(t,euler(:,1)); hold on; plot(t,euler(:,2),'r'); plot(t,euler(:,3),'g'); legend('\psi','\phi','\theta'); grid;
        title('Angulos de Euler')
end

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
