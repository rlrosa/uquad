function plot_w(w_control)

figure()
    plot(w_control)
    axis([2 length(w_control) min(min(w_control(2:end,:))) ...
        max(max(w_control(2:end,:)))])
    title('w motores')
    legend('adelante','izquierda','atras','derecha')