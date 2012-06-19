function [aconv,wconv,euler,mconv] = mong_conv(a,w,m,plotear,t_imu,T)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [aconv,wconv,euler,mconv] = mong_conv(a,w,m,plotear,t_imu,T)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if(~exist('plotear','var'))
  plotear = 1;
end

AT=load('acc_temp','x','to');
a_to = AT.to;
GT=load('gyro_temp','x','to');
g_to = GT.to;

if(~exist('t_imu','var'))
%   fprintf('Calibración sin compensación por temperatura\nSe debe pasar como último parámetro el vector de temperaturas\n');
  t_imu = a_to*ones(length(a));
else
  t_imu = t_imu/10;
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

Ka_1 = (Ka^-1);
               
aconv=zeros(size(a));
for i=1:length(a(:,1))
    bat = ba + [0;0; AT.x(1)]*(t_imu(i)-a_to);
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
    bgt =bg+[GT.x(1);GT.x(2);GT.x(3)]*(t_imu(i)-g_to)+[GT.x(4);GT.x(5);GT.x(6)];
    auxg=Tg*(Kg^(-1))*(w(i,:)'-bgt);
    wconv(i,:)=auxg';
end

% MAGNETOMETRO

mconv = zeros(size(m));
euler = zeros(size(m));
n_vueltas=0;
for i=1:length(m(:,1))
    auxm = Km*(m(i,:)'-bm);
    mconv(i,:)=auxm';
    
    % Convierto a angulos de Euler
    % phi, psi, theta en radianes
    if (abs(aconv(i,1))<9.72/9.81*norm(aconv(i,:)))
        phi=-asin(aconv(i,1)/norm(aconv(i,:)));
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
    theta=-atan2(mrot(2),mrot(1))+0.17069; %9.78@deg;
    
%     if ((i>1) && (abs(theta-euler(i-1,3)) >= pi))
%         theta = theta-fix((theta-euler(i-1,3)-pi)/(2*pi))*2*pi;
%     end
    
    euler(i,:)=[  ...
      psi, ...
      phi, ...
      theta
      ];
  
end  

% k=0;
% theta = zeros(size(euler(:,3)));
% for i=3:length(euler(:,3))
%     if (euler(i-1,3) < -pi/2) &  (euler(i,3) > pi/2)
%         k=k-1;
%     elseif (euler(i-1,3) > pi/2) &  (euler(i,3) < -pi/2)
%         k = k+1;
%     elseif (euler(i-2,3) < -pi/2) &  (euler(i-1,3) < pi/2) &(euler(i,3) > pi/2)
%         k=k-1;
%     elseif (euler(i-2,3) > pi/2) & (euler(i-1,3) > -pi/2) & (euler(i,3) < -pi/2)
%     k = k+1;
%     end
% 
%     theta(i,3) = 2*pi*k+euler(i,3);
% end

% euler(:,3) = theta(:,3);

if plotear
    if(~exist('T','var'))
      T=1/fs;
      t=0:T:T*(length(a(:,1))-1);
    else
      if(length(T) >= length(a(:,1)))
        fprintf('WARN: will trim T (%d) to match length(a) (%d)\n', length(T), length(a(:,1)));
        t = T(1:length(a(:,1)));
      elseif (length(T) <= length(a(:,1)))
        fprintf('WARN: will trim data (%d) to match length(T) (%d)\n', length(T), length(a(:,1)));
        aconv=aconv((1:length(T)),:);
        wconv=wconv((1:length(T)),:);
        euler=euler((1:length(T)),:);
        t=T;
      end
    end
    figure()
        subplot(311)
        plot(t,aconv(:,1)); hold on; plot(t,aconv(:,2),'r'); plot(t,aconv(:,3),'g'); legend('a_x','a_y','a_z'); grid;
        title('Aceleraciones lineales en m/(s^2)')
        subplot(312)
        plot(t,wconv(:,1)); hold on; plot(t,wconv(:,2),'r'); plot(t,wconv(:,3),'g'); legend('w_x','w_y','w_z'); grid;
        title('Velocidades angulares en rad/s')
        subplot(313)
        plot(t,euler(:,1)*180/pi); hold on; plot(t,euler(:,2)*180/pi,'r'); plot(t,euler(:,3)*180/pi,'g'); legend('\psi','\phi','\theta'); grid;
        title('Angulos de Euler en grados')
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
