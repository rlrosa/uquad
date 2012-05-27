function []=rc(who,t_rc,x_hat_rc,z)

green1 = [154,205,50]/256;
green2 = [34,139,34]/256;
blue1  = [0,0,128]/256;
blue2  = [0,0,255]/256;
red1   = [255,0,0]/256;
red2   = [178,34,34]/256;
width  = 1.5;

% who     = 'y';  % 'x', 'y', o 'z'
% program = 'km'; % 'km' o 'c';

if who=='x'
    k=1;
    angulo = '\psi';
elseif who == 'y'
    k=2;
    angulo = '\phi';
elseif who == 'z';
    k=3;
    angulo = '\theta';
end

% if strcmp(program,'km')
%     x_hat_rc = x_hat;
%     t_rc = T;
% elseif strcmp(program,'c')
%     x_hat_rc = x_hat_c;
%     t_rc = t;
% end

%% Giros a mano - Motores apagados

figure('name',['giro en ' who ': ' angulo])

subplot(211)
    hold on; grid
    plot(t_rc,180/pi*z(:,k),'*-','color',blue1,'markersize',width)
    plot(t_rc,180/pi*x_hat_rc(1:end,k+3),'color',green2,'linewidth',3)
    handle = legend(['\fontsize{15}' angulo '_{imu}'],['\fontsize{15}' angulo '_{kalman}']);
    set(handle, 'Box', 'off','location','northwest');
    title('\fontsize{15}Orientacion [^o]')
    xlabel('\fontsize{12}Tiempo desde ./main  [s]');
    ylabel('\fontsize{12}Angulos de Euler [^o]');
    line([t_rc(1) t_rc(end)],[180/pi*x_hat_rc(1,6) 180/pi*x_hat_rc(1,6)],'color','black')
    axis tight
    hold off

subplot(212)
hold on; grid    
    plot(t_rc,z(:,k+6),'*-','color',blue1,'markersize',width)    
    plot(t_rc,x_hat_rc(1:end,k+9),'color',green2,'linewidth',width)
    handle = legend(['\fontsize{15}w_{q' who '}_{imu}'],['\fontsize{15}w_{q' who '}_{kalman}']);
    set(handle, 'Box', 'off','location','northwest');
    title('\fontsize{15}\omega_q [rad/s]');
    xlabel('\fontsize{12}Tiempo desde ./main  [s]');
    ylabel('\fontsize{12}Velocidades angulares [rad/s]');
    axis tight
    hold off


    
    