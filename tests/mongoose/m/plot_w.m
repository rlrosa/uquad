function plot_w(w)

green  = [34,139,34]/256;
yellow = [218,165,32]/256;
blue   = [0,0,128]/256;
red    = [178,34,34]/256;
width  = 2;

t = w(:,1);
if(mean(t) < .1)
  % logs con delta tiempos
  t(1) = 0; % bug, era enorme
  t = cumsum(t);
end
w_control = w(:,2:end);

figure()
    hold on
    plot(t, w_control(:,1),'color',blue,'linewidth',width)
    plot(t, w_control(:,2),'color',red,'linewidth',width)
    plot(t, w_control(:,3),'color',green,'linewidth',width)
    plot(t, w_control(:,4),'color',yellow,'linewidth',width)
    axis([t(1) t(end) min(min(w_control(2:end,:))) - 2 ...
        max(max(w_control(2:end,:))) + 2])
    title('\fontsize{16}Velocidad angular de los motores');
    handle = legend('adelante','izquierda','atras','derecha');
    set(handle, 'Box', 'off','location','northwest');
    xlabel('\fontsize{13}Tiempo desde ./main [s]')
    ylabel('\fontsize{13}\omega motores [rad/s]')
    grid
    hold off
    
    