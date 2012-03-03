function [val,reg,D] = read_MSP_log(archivo)

F = fopen(archivo);
D = textscan(F,'%s','delimiter',' ');
fclose(F);

reg = hex2dec(D{1}(1:2:end));
val = hex2dec(D{1}(2:2:end));

figure
    plot(val,'*-')
    title('\fontsize{16}Valor recibido por el MSP430F5438')
    xlabel('\fontsize{16}Valor recibido')
    ylabel('\fontsize{16}Muestras')
%     hold on
%     plot(92/max(reg)*reg,'r+')
figure
    plot(reg,'r*')
    title('\fontsize{16}Registro recibido por el MSP430F5438')
    xlabel('\fontsize{16}Registro recibido')
    ylabel('\fontsize{16}Muestras')
    axis([0 length(reg) 159 163])
    