do_plot = 1;
no_indice = 0;
modo = 2;
avg_size = 2;
if(mod(avg_size,2)~=0)
  error('quiero avg_size par!')
end

tmp = pwd;
if(~strcmp(tmp(end-13:end),'barometro/data'))
  error('estas en la carpeta equivocada! Anda a la carpeta barometro/data')
end

%% teo
alt_teo = load('l0/README'); % en cm
delta_alt = max(alt_teo) - min(alt_teo);

%% cuentas
[alt,ind,temp,pres] = barom_multi_log('l0_rapido/',0,0,0);
puntos_de_int = 3; % init, medio y fin
err_diff = zeros(puntos_de_int, length(ind));
err_acum = zeros(puntos_de_int, length(ind));
alt_stats = zeros(puntos_de_int, length(ind));
alt = alt*100; % cm
figure()
title(sprintf('avg_size: %d',avg_size))
subplot 221
for j = 1:length(ind)
  if(j==1)
    tmp_alt = alt(1:ind(1));
  else
    tmp_alt = alt(ind(j-1):ind(j));
  end
  tmp = vec2mat(tmp_alt,2^modo);
  tmp = mean(tmp(1:end-1,:),2);
  if(no_indice > 0)
    tmp = tmp(1:no_indice);
  end 
  ind_tmp = find(tmp<=min(tmp));
  % specs
  alt_stats(1,j) = mean( ...
    tmp(1:avg_size) ...
    );
  alt_stats(2,j) = mean( ...
    tmp(ind_tmp(1)-avg_size/2+1:ind_tmp(1) + avg_size/2) ...
    );
  alt_stats(3,j) = mean( ...
      tmp(end-avg_size+1:end) ...
      );
  subplot(220 + j)
  hold on
  if(modo ~= 0)
    plot(tmp_alt-3800);
    plot(2^modo*[0:length(tmp)-1],tmp-3800,'g.-')
  end
  xlabel('Muestras','Fontsize',16)
  ylabel('Altura (cm)','Fontsize',16)
  title(sprintf('Experimento #%d',j),'Fontsize',16)
  set(gca,'Fontsize',16)
  plot(2^modo*[1 ind_tmp(1) length(tmp)]', ...
    alt_stats(:,j) - 3800, ...
    'r.','MarkerSize',20)
  grid on
  axis tight
  hold off
end

legend('DatC: Datos crudos', ...
  sprintf('DatM: Datos de a %d',2^modo), ...
  sprintf('Max (Min): Promedio de \n%d muestra(s) de DatM)',avg_size-1))

%% Error al volver al origen
% por cuanto "no" volvi
err = zeros(puntos_de_int,2);
tmp = alt_stats(1,:) - alt_stats(3,:);
err(1,:) = [mean(tmp) std(tmp)];

%% Error en la altura de la estantería - bajando
tmp = alt_stats(1,:) - alt_stats(2,:) - delta_alt;
err(2,:) = [mean(tmp) std(tmp)];

%% Error en la altura de la estantería - subiendo
tmp = alt_stats(3,:) - alt_stats(2,:) - delta_alt;
err(3,:) = [mean(tmp) std(tmp)];

%% pseudo-estadistica
descrip = {'bajada y subida' 'ida' 'vuelta'};
fprintf('-- -- -- -- -- -- -- -- -- -- -- -- -- -- --\n')
for i = 1:puntos_de_int
  fprintf('%s\n',descrip{i})
  fprintf('\tError promedio:\t%0.4f cm\n\tSigma:\t\t%0.4f cm\n\n', ...
    err(i,1),err(i,2))
end
fprintf('-- -- -- -- -- -- -- -- -- -- -- -- -- -- --\n')