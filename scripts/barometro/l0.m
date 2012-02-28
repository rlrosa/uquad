do_plot = 1;
no_indice = 0;
modo = 0;

tmp = pwd;
if(~strcmp(tmp(end-13:end),'barometro/data'))
  error('estas en la carpeta equivocada! Anda a la carpeta barometro/data')
end

%% logs - bajada/subida estanteria
% logs 5 seg
%logs = {'l0_rapido/bajada-1' 'l0_rapido/subida-1'};
% logs 1 m
logs = {'l0/'};

%% teo
alt_teo = load('l0/README'); % en cm

%% cuentas
escalones = 5;
err_diff = zeros(escalones-1,length(logs));
err_acum = zeros(escalones,length(logs));
alt_mean = zeros(escalones,length(logs));
for i = 1:length(logs)
  if(strcmp(logs{i},'l0/'))
    raw = 1;
  else
    raw = 0;
  end
  [alt,ind,temp,pres] = barom_multi_log( ...
    sprintf('%s/',logs{i}),0,1,raw);
  alt = alt*100; % cm
  if(length(ind) ~= escalones )
    error('no encontre suficientes escalones!');
  end
  tmp = vec2mat(alt(1:ind(1)),2^modo);
  tmp = mean(tmp,2);
  if(no_indice > 0)
    tmp = tmp(1:no_indice);
  end
  alt_mean(1,i) = mean(tmp);
  for j = 2:length(ind)
    tmp = vec2mat(alt(ind(j-1):ind(j)),2^modo);
    tmp = mean(tmp,2);
    if(no_indice > 0)
      tmp = tmp(1:no_indice);
    end
    alt_mean(j,i) = mean(tmp);
  end
  alt_teo = alt_teo - alt_teo(1) + alt_mean(1,i); % tomo en el 0 en la primer muestra
  err_diff(:,i) = diff(alt_teo) - diff(alt_mean(:,i));
  err_acum(:,i) = alt_mean(:,i) - alt_teo;
end

%% error acum
if(do_plot)
  figure
  plot(abs(err_acum),'.-')
  legend('Subida 1','Subida 2', 'Fontsize',16,'Location','NorthWest')
  title('Error acumulado respecto a la cinta metrica','Fontsize',16)
  set(gca,'Fontsize',16)
  xlabel('Puntos','Fontsize',16)
  ylabel('Error respecto a la cinta graduada (cm)','Fontsize',16)
  grid on
end
%% error rel
if(do_plot)
  figure
  plot(abs(err_diff),'.-')
  legend('bajada-1','subida-1', 'Fontsize',16,'Location','NorthWest')
  title('Error entre puntos','Fontsize',16)
  set(gca,'Fontsize',16)
  xlabel('Puntos','Fontsize',16)
  ylabel('Error respecto a la distancia teo. entre puntos (cm)','Fontsize',16)
  grid on
end
%% pseudo-estadistica
err_diff_vec = reshape(err_diff,length(err_diff)*length(logs),1);
% err_diff_vec = err_diff(:,2); % analizar series de a una
err_mu = mean(err_diff_vec);
err_sigma = std(err_diff_vec);
fprintf('-- -- -- -- -- -- -- -- -- -- -- -- -- -- --\n')
fprintf('Entre puntos deberia medir cerca de 30 cm\n')
fprintf('\tError promedio:\t%0.4f cm\n\tSigma:\t\t%0.4f cm\n\n', ...
  err_mu,err_sigma)
fprintf('-- -- -- -- -- -- -- -- -- -- -- -- -- -- --\n')
