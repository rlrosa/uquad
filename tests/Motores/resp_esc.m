close all
clear all
clc

F = fopen('respuesta_escalon.vcd');
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

figure
    bar(t,data)
    axis([0 max(t) -1 2])

subidas = t(1:2:end)*1e-8;
rise = diff(subidas);
diff_min=mean(rise(50:end));

figure
    plot(subidas(2:end), pi.*1./rise)
    title('\fontsize{16}Respuesta al escalón de los motores')
    xlabel('\fontsize{16}Tiempo (s)')
    ylabel('\fontsize{16}Velocidad angular (rad/s)')

