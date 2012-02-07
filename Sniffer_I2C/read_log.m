% close all
% clear all
% clc

function [dat,D0_val,D2_val,D4_val,D6_val,...
          D0_add,D2_add,D4_add,D6_add] = read_log(archivo)

D0_val=0;D2_val=0;D4_val=0;D6_val=0;
D0_add=0;D2_add=0;D4_add=0;D6_add=0;
      
% prueba  = '5';
% intento = '2';
% F = fopen(['./logs/',prueba,intento,'.vcd']);
F = fopen(archivo);
D = textscan(F,'%s','delimiter','\t');
fclose(F);

j=1;
for i = 1:size(D{1},1)
    if (D{1}{i}(1) == 'b') % && (D{1}{i}(2) ~= 'x')
        data{j} = (D{1}{i}(2:9));
        j=j+1;
    end
end

i=1; j=1; k=1; l=1; m=1;
while i<size(data,2)
    if data{i} == '11010010'
        if data{i+1}(2) ~= 'x'
            D2_add (j) = bin2dec(data{i+1});
        else
            D2_add (j) = 0;
        end
        if data{i+2}(2) ~= 'x'
            D2_val (j) = bin2dec(data{i+2});
        else
            D2_val (j) = 0;
        end
        j=j+1;
    elseif data{i} == '11010100'
        if data{i+1}(2) ~= 'x'
            D4_add (k) = bin2dec(data{i+1});
        else
            D4_add (k) = 0;
        end
        if data{i+2}(2) ~= 'x'
            D4_val (k) = bin2dec(data{i+2});
        else
            D4_val (k) = 0;
        end
        k=k+1;
    elseif data{i} == '11010110'
        if data{i+1}(2) ~= 'x'
            D6_add (l) = bin2dec(data{i+1});
        else
            D6_add (l) = 0;
        end
        if data{i+2}(2) ~= 'x'
            D6_val (l) = bin2dec(data{i+2});
        else
            D6_val (l) = 0;
        end
        l=l+1;
        elseif data{i} == '11010000'
        if data{i+1}(2) ~= 'x'
            D0_add (m) = bin2dec(data{i+1});
        else
            D0_add (m) = 0;
        end
        if data{i+2}(2) ~= 'x'
            D0_val (m) = bin2dec(data{i+2});
        else
            D0_val (m) = 0;
        end
        m=m+1;
    end
    i=i+3;
end

j=1;
for i=1:length(data)
    if data{i}(1) ~='x'
        dat(j)=bin2dec(data{i});
        j=j+1;
    end
end


%% PLOTS

if (length(D0_val~=1) && length(D2_val)~=1 && length(D4_val)~=1 && length(D6_val)~=1)

% figure('Name',['Prueba ',prueba,', Intento ',intento])
figure('Name','Potencia')
subplot 221
    plot(D0_val,'b*')
%     axis([1 length(D0_val) 0 100])
    title('D0')
    xlabel('Muestras')
    ylabel('Información envidada')
subplot 222
    plot(D2_val,'y>')
%     axis([1 length(D0_val) 0 100])
    title('D2')
    xlabel('Muestras')
    ylabel('Información envidada')
subplot 223
    plot(D4_val,'ro')
%     axis([1 length(D0_val) 0 100])
    title('D4')
    xlabel('Muestras')
    ylabel('Información envidada')
subplot 224
    plot(D6_val,'gs')
%     axis([1 length(D0_val) 0 100])
    title('D6')
    xlabel('Muestras')
    ylabel('Información envidada')

end

if (length(D0_add)~=1 && length(D2_add)~=1 && length(D4_add)~=1 && length(D6_add)~=1)
    
figure('Name','Registro')
subplot 221
    plot(D0_add,'b*')
%     axis([1 length(D0_add) 0 100])
    title('D0')
    xlabel('Muestras')
    ylabel('Registro del esclavo')
subplot 222
    plot(D2_add,'y>')
%     axis([1 length(D2_add) 0 100])
    title('D2')
    xlabel('Muestras')
    ylabel('Registro del esclavo')
subplot 223
    plot(D4_add,'ro')
%     axis([1 length(D4_add) 0 100])
    title('D4')
    xlabel('Muestras')
    ylabel('Registro del esclavo')
subplot 224
    plot(D6_add,'gs')
%     axis([1 length(D6_add) 0 100])
    title('D6')
    xlabel('Muestras')
    ylabel('Registro del esclavo')
    
end    
  
% figure
% plot(D0_val,'b*')
% hold on
% plot(D2_val,'y>')
% plot(D4_val,'ro')
% plot(D6_val,'gs')