close all
clear all
clc

usar_temp = 0;
avg_size = 20;


files = {'subida_1-2012_02_21_xx_22_01_45.log' ...
         'subida_2-2012_02_21_xx_22_32_29.log' ...
         'bajada_1-2012_02_21_xx_22_19_33.log' ...
         'bajada_express-2012_02_21_xx_22_43_19.log'};

%% SUBIDA 1, SUBIDA 2, BAJADA
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
  hm_tmp = zeros(size(hp_tmp));
  j=1;
  for i=1:2:length(ind)
      % ploteo el valor medido con la cinta metrica
      line([ind(i) ind(i+1)],[hp_tmp(j) hp_tmp(j)], ...
        'color','red','LineWidth',2,'Parent',ax_plot)
      % me queda con la altura experimental
      hm_tmp(j)=mean(alt(ind(i):ind(i+1)));
      j=j+1;
  end
  
  legend(ax_plot,'Datos crudos', ...
    sprintf('Promedio de %d muestras',avg_size), ...
    'Cinta metrica')

  eval(sprintf('hp%d=hp_tmp;',k));
  eval(sprintf('hm%d=hm_tmp;',k));
end

%% Bajada express
% no hay tiempo de init y tiempo de fin en cada piso, hay solo una marca de
% tiempo por piso, por eso hay que armar un procedimiento diferente.
k=4;
[alt,temp,ind,~] = barom(sprintf('barometro/data/l2/%s',files{k}),0,0);
if(usar_temp)
  ax_plot = barom_plot(alt,avg_size,ind,temp);
else
  ax_plot = barom_plot(alt,avg_size,ind);
end
title('Bajada rapido');
if(k==3 || k==4 )
  hp=(load('barometro/data/l2/README')); % alturas posta
else
  hp=flipud(load('barometro/data/l2/README')); % alturas posta
end
if(ind(1)-avg_size/2<1 || ind(1)+avg_size/2  > length(alt))
  fprintf('No me alcanzan las muestras para armar un promedio de ref.\nMe mato.\n')
  return;
end
hp_tmp = hp+mean(alt(ind(1)-avg_size/2:ind(1)+avg_size/2))-hp(1);
hm_tmp = zeros(size(hp_tmp));
hold on;
for i=1:length(ind)
    plot(ind(i),hp_tmp(i),'r.','MarkerSize',15,'Parent',ax_plot);
    hm_tmp(i,1)=mean(alt(ind(i)-avg_size/2:ind(i)+avg_size/2));
end

legend(ax_plot,'Datos crudos', ...
  sprintf('Promedio de %d muestras',avg_size), ...
  'Cinta metrica')

eval(sprintf('hp%d=hp_tmp;',k));
eval(sprintf('hm%d=hm_tmp;',k));

%% Numeritos
fprintf('Diferencia en las alturas\n\nSubida 1\n')
disp(hm1-hp1)
fprintf('\nSubida 2\n')
disp(hm2-hp2)
fprintf('\nBajada\n')
disp(hm3-hp3)
fprintf('\nBajada express\n')
disp(hm4-hp4)


%% Error acumulado
figure()
plot([0:length(hm1)-1]',abs(hm1-hp1),'b.-')
hold on
plot([0:length(hm2)-1]',abs(hm2-hp2),'r.-')
plot([0:length(hm3)-1]',abs(hm3-hp3),'g.-')
plot([0:length(hm4)-1]',abs(hm4-hp4),'k.-')
legend('Subida 1','Subida 2','Bajada','Bajada rapida', ...
  'Fontsize',16,'Location','NorthWest')
title('Error acumulado respecto a la cinta metrica','Fontsize',16)
set(gca,'Fontsize',16)
xlabel('# de piso','Fontsize',16)
ylabel('Error respecto a la cinta metrica (m)','Fontsize',16)
grid on

%% Error entre pisos
figure()
plot(abs(diff(hm1-hp1)),'b.-')
hold on
plot(abs(diff(hm2-hp2)),'r.-')
plot(abs(diff(hm3-hp3)),'g.-')
plot(abs(diff(hm4-hp4)),'k.-')
legend('Subida 1','Subida 2','Bajada','Bajada rapida', ...
  'Fontsize',16,'Location','NorthWest')
title('Error en la altura de un piso a otro respecto a la cinta metrica','Fontsize',16)
set(gca,'Fontsize',16)
xlabel('# de piso','Fontsize',16)
ylabel('Error respecto a la cinta metrica (m)','Fontsize',16)
grid on

%% Error entre pisos (%)
figure()
plot(abs(diff(hm1-hp1))./diff(hp1)*100,'b.-')
hold on
plot(abs(diff(hm2-hp2)./diff(hp1)*100),'r.-')
plot(abs(diff(hm3-hp3)./diff(hp1)*100),'g.-')
plot(abs(diff(hm4-hp4)./hp4(2:end)*100),'k.-')
legend('Subida 1','Subida 2','Bajada','Bajada rapida', ...
  'Fontsize',16,'Location','NorthWest')
title('Error en la altura de un piso a otro respecto a la cinta metrica (%)','Fontsize',16)
set(gca,'Fontsize',16)
xlabel('# de piso','Fontsize',16)
ylabel('Error/Cinta\_metrica*100','Fontsize',16)
grid on
