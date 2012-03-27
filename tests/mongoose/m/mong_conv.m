function [aconv,wconv,euler] = mong_conv(a,w,m,plotear,t_imu)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [aconv,wconv,mconv] = mong_conv(a,w,m,plotear)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if(~exist('plotear','var'))
  plotear = 1;
end

AT=load('acc_temp','x','to');
a_to = AT.to;

if(~exist('t_imu','var'))
  fprintf('Flaco, mirá que no pasaste la temperatura, y eso q te avise\nSe calibrará sin la compensación por temperatura\n');
  t_imu = a_to;
else
    if length(t_imu)>512
        t_imu = mean(t_imu(1:512))/10;
    else
        t_imu = mean(t_imu)/10;
    end
end

fs=100;

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

M=load('mag');
Km=M.K;
bm = M.b;
%% Conversion

% ACELEROMETRO
Ka=[kax 0 0;
    0 kay 0;
    0 0 kaz];

ba=[bax; bay; baz];

Ta=[1 -ayza azya;
   axza 1 -azxa;
   -axya ayxa 1];

Ka_1 = (Ka^-1) + [ AT.x(1)*(t_imu-a_to) 0                    0 ;                      ...
                   0                    AT.x(2)*(t_imu-a_to) 0 ;                      ...
                   0                    0                    AT.x(3)*(t_imu-a_to) ] ;
               
bat = ba + [AT.x(4); AT.x(5); AT.x(6)]*(t_imu-a_to);

aconv=zeros(size(a));
for i=1:length(a(:,1))
    aux=Ta*(Ka_1)*(a(i,:)'-bat);
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
    wconv(i,:)=pi/180*auxg';
end


% MAGNETOMETRO

mconv = zeros(size(m));
euler = zeros(size(m));
for i=1:length(m(:,1))
    auxm = Km*(m(i,:)'-bm);
    mconv(i,:)=auxm';
    
    % Convierto a angulos de Euler
    % phi, psi, theta en radianes
    if (abs(aconv(i,1))<9.72)
        phi=-asin(aconv(i,1)/9.81);
        psi=atan2(aconv(i,2),aconv(i,3));
    elseif aconv(i,1) > 0
        phi=-pi/2;
        psi=0;
    else 
        phi=pi/2;
        psi=0;
    end
    mrot = [ cos(phi)/(cos(phi)^2 + sin(phi)^2), (sin(phi)*sin(psi))/((cos(phi)^2 + sin(phi)^2)*(cos(psi)^2 + sin(psi)^2)), (cos(psi)*sin(phi))/((cos(phi)^2 + sin(phi)^2)*(cos(psi)^2 + sin(psi)^2));
                                                 0,                                           cos(psi)/(cos(psi)^2 + sin(psi)^2),                                         -sin(psi)/(cos(psi)^2 + sin(psi)^2);
            -sin(phi)/(cos(phi)^2 + sin(phi)^2), (cos(phi)*sin(psi))/((cos(phi)^2 + sin(phi)^2)*(cos(psi)^2 + sin(psi)^2)), (cos(phi)*cos(psi))/((cos(phi)^2 + sin(phi)^2)*(cos(psi)^2 + sin(psi)^2))]...
            *auxm;
    mrot = mrot*180/pi; % atan2 toma radianes como argumento.
    theta=atan2(mrot(1),mrot(2))+0.17069; %9.78@deg;
    euler(i,:)=[  ...
      psi, ...
      phi, ...
      theta];
end

if plotear
    T=1/fs;
    t=0:T:T*(length(a(:,1))-1);
    figure()
        subplot(311)
        plot(t,aconv(:,1)); hold on; plot(t,aconv(:,2),'r'); plot(t,aconv(:,3),'g'); legend('a_x','a_y','a_z'); grid;
        title('Aceleraciones lineales en m/(s^2)')
        subplot(312)
        plot(t,wconv(:,1)); hold on; plot(t,wconv(:,2),'r'); plot(t,wconv(:,3),'g'); legend('w_x','w_y','w_z'); grid;
        title('Velocidades angulares en rad/s')
        subplot(313)
        plot(t,euler(:,1)*180/pi); hold on; plot(t,euler(:,2)*180/pi,'r'); plot(t,euler(:,3)*180/pi,'g'); legend('\psi','\phi','\theta'); grid;
        title('Angulos de Euler en radianes')
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
