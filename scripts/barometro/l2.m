close all
clear all
clc

%% Subidas

% SUBIDA 1

[alt,temp,ind,ax_plot] = barom('barometro/data/l2/subida_1-2012_02_21_xx_22_01_45.log',1,0);
title('Subida 1')
hp=flipud(load('barometro/data/l2/README')); % alturas posta
hp1=hp+mean(alt(ind(1):ind(2)))-hp(1);
j=1;
for i=1:2:length(ind)
    line([ind(i) ind(i+1)],[hp1(j) hp1(j)],'color','red','Parent',ax_plot)
    hm1(j,1)=mean(alt(ind(i):ind(i+1)));
    j=j+1;
end

% SUBIDA 2

[alt,temp,ind,ax_plot] = barom('barometro/data/l2/subida_2-2012_02_21_xx_22_32_29.log',1,0);
title('Subida 2')
hp=flipud(load('barometro/data/l2/README')); % alturas posta
hp2=hp+mean(alt(ind(1):ind(2)))-hp(1);
j=1;
for i=1:2:length(ind)
    line([ind(i) ind(i+1)],[hp2(j) hp2(j)],'color','red','Parent',ax_plot)
    hm2(j,1)=mean(alt(ind(i):ind(i+1)));
    j=j+1;
end

%% Bajada

[alt,temp,ind,ax_plot] = barom('barometro/data/l2/bajada_1-2012_02_21_xx_22_19_33.log',1,0);
title('Bajada 2')
hp=(load('barometro/data/l2/README')); % alturas posta
hp3=hp+mean(alt(ind(1):ind(2)))-hp(1);
j=1;
for i=1:2:length(ind)
    line([ind(i) ind(i+1)],[hp3(j) hp3(j)],'color','red','Parent',ax_plot)
    hm3(j,1)=mean(alt(ind(i):ind(i+1)));
    j=j+1;
end

fprintf('Diferencia en las alturas\n\nSubida 1\n')
disp(hm1-hp1)
fprintf('\nSubida 2\n')
disp(hm2-hp2)
fprintf('\nBajada\n')
disp(hm3-hp3)

figure()
plot(abs(hm1-hp1))
hold on
plot(abs(hm2-hp2),'r')
plot(abs(hm3-hp3),'g')
legend('Subida 1','Subida 2','Bajada')
title('Error en valor absoluto de la altura')
