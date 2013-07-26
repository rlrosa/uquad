cd ccclose all
clear all
clc

F = fopen('escalon_dos.vcd');
D = textscan(F,'%s','delimiter','\n');
fclose(F);

taux = D{1}(1:2:end);
t = zeros(length(taux),1);
for i=1:length(taux)
    t(i)=str2num(taux{i}(2:end));
end

daux = D{1}(2:2:end);
data = zeros(length(daux),1);
for i=1:length(daux)
    data(i)=str2num(daux{i}(1));
end

subidas = t(1:2:end)*1e-8;  % Tiempos de los flancos de subida
subidas = subidas(diff(subidas)>3e-3); % Para eliminar ruidos
rise    = diff(subidas);    % Diferencia entre tiempos de los flacos de subida
w       = [0;pi./rise];     % Velocidad angular de giro (rad/seg)

wmax    = mean(w(250:end));
wmin    =mean(w(1:100));
ind     = find(w>0.9*(wmax-wmin)+wmin,1,'first');
ind2    = find(w<wmin+0.1*(wmax-wmin),1,'last');
rise_t  = subidas(ind+1)-subidas(ind2+1);

figure
    bar(t*1e-8,data)
    axis([0 max(t*1e-8) -0.5 1.5])
    title('\fontsize{16}Salida del analizador lógico')
    xlabel('\fontsize{16}Tiempo (s)')
    ylabel('\fontsize{16}Valor lógico')

figure
    plot(subidas(1:end), w(1:end))
    title('\fontsize{16}Respuesta al escalón de los motores')
    xlabel('\fontsize{16}Tiempo (s)')
    ylabel('\fontsize{16}Velocidad angular (rad/s)')
    hold on
    plot(subidas(1:end),(0.9*(wmax-wmin)+wmin)*ones(length(w),1),'r--')
    text(subidas(ceil(0.5*length(subidas))),0.93*(wmax-wmin)+wmin,'\fontsize{12}90% del valor en régimen')
    plot(subidas(1:end),(0.1*(wmax-wmin)+wmin)*ones(length(w),1),'r--')
    text(subidas(ceil(0.5*length(subidas))),0.13*(wmax-wmin)+wmin,'\fontsize{12}10% del valor en régimen')
    line([subidas(ind) subidas(ind)],[0 wmax],'color','green')
    line([subidas(ind2) subidas(ind2)],[0 wmax],'color','green')
    text(subidas(1)-.1,0.5*wmax,['\fontsize{13}t_{rise} = ',num2str(rise_t),' s']);
    axis([-0.2 5 100 330])

