close all
clear all
clc

usar_temp = 0;
avg_size = 20;

%% SUBIDA 1, SUBIDA 2, BAJADA

files = {'subida_1-2012_02_21_xx_22_01_45.log' ...
         'subida_2-2012_02_21_xx_22_32_29.log' ...
         'bajada_1-2012_02_21_xx_22_19_33.log'};

for k = 1:3
  [alt,temp,ind,~] = barom(sprintf('barometro/data/l2/%s',files{k}),0,0);
  if(usar_temp)
    ax_plot = barom_plot(alt,avg_size,ind,temp);
  else
    ax_plot = barom_plot(alt,avg_size,ind);
  end
  title(sprintf('%s',files{k}(1:8)));
  if(k==3)
    hp=(load('barometro/data/l2/README')); % alturas posta
  else
    hp=flipud(load('barometro/data/l2/README')); % alturas posta
  end
  hp_tmp=hp+mean(alt(ind(1):ind(2)))-hp(1);
  j=1;
  for i=1:2:length(ind)
      line([ind(i) ind(i+1)],[hp_tmp(j) hp_tmp(j)], ...
        'color','red','LineWidth',2,'Parent',ax_plot)
      hm_tmp(j,1)=mean(alt(ind(i):ind(i+1)));
      j=j+1;
  end
  
  legend(ax_plot,'Datos crudos', ...
    sprintf('Promedio de %d muestras',avg_size), ...
    'Cinta metrica')

  eval(sprintf('hp%d=hp_tmp;',k));
  eval(sprintf('hm%d=hm_tmp;',k));
end

fprintf('Diferencia en las alturas\n\nSubida 1\n')
disp(hm1-hp1)
fprintf('\nSubida 2\n')
disp(hm2-hp2)
fprintf('\nBajada\n')
disp(hm3-hp3)

%% Error absoluto
figure()
plot(abs(hm1-hp1),'b.-')
hold on
plot(abs(hm2-hp2),'r.-')
plot(abs(hm3-hp3),'g.-')
legend('Subida 1','Subida 2','Bajada','Fontsize',16,'Location','NorthWest')
title('Error en respecto a la cinta metrica','Fontsize',16)
set(gca,'Fontsize',16)
xlabel('# de piso','Fontsize',16)
ylabel('Error respecto a la cinta metrica (m)','Fontsize',16)
grid on

%% Error relativo
% figure()
% plot(abs(hm1-hp1)./hp1*100,'b.-')
% hold on
% plot(abs(hm2-hp2)./hp2*100,'r.-')
% plot(abs(hm3-hp3)./hp3*100,'g.-')
% legend('Subida 1','Subida 2','Bajada','Fontsize',16,'Location','NorthWest')
% title('Error relativo','Fontsize',16)
% set(gca,'Fontsize',16)
% xlabel('# de piso','Fontsize',16)
% ylabel('Error/Altura\_cinta*100','Fontsize',16)
% grid on

