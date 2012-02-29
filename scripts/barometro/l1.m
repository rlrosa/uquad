do_plot = 0;
no_indice = 0;
modo = 0;

tmp = pwd;
if(~strcmp(tmp(end-1:end),'l1'))
  error('estas en la carpeta equivocada! Anda a la carpeta l1!')
end

%% logs
logs = {'esc1' 'esc2'};

%% teo
step_teo = [0:1:5]'; % en m

%% cuentas
escalones = 6;
err_diff = zeros(escalones-1,length(logs));
err_acum = zeros(escalones,length(logs));
alt_mean = zeros(escalones,length(logs));
for i = 1:length(logs)
  [alt,ind,temp,pres] = barom_multi_log( ...
    sprintf('%s/',logs{i}),0,0,1);
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
  alt_teo = step_teo + alt_mean(1,i); % tomo en el 0 en la primer muestra
  err_diff(:,i) = diff(alt_mean(:,i)); % deberian dar 1 metro entre si
  err_acum(:,i) = alt_mean(:,i) - alt_teo;
end

%% error acum
if(do_plot)
  figure
  plot(abs(err_acum),'.-')
  legend('Subida 1','Subida 2', 'Fontsize',16,'Location','NorthWest')
  title('Error acumulado respecto a la cuerda graduada','Fontsize',16)
  set(gca,'Fontsize',16)
  xlabel('Puntos','Fontsize',16)
  ylabel('Error respecto a la cuerda graduada (m)','Fontsize',16)
  grid on
end
%% error rel
if(do_plot)
  figure
  plot(abs(1-err_diff),'.-')
  legend('Subida 1','Subida 2', 'Fontsize',16,'Location','NorthWest')
  title('Error entre puntos','Fontsize',16)
  set(gca,'Fontsize',16)
  xlabel('Puntos)','Fontsize',16)
  ylabel('Error respecto a la distancia teo. entre puntos(m)','Fontsize',16)
  grid on
end
%% pseudo-estadistica
err_diff_vec = reshape(err_diff,length(err_diff)*length(logs),1);
% err_diff_vec = err_diff(:,2); % analizar series de a una
err_mu = mean(err_diff_vec - 1);
err_sigma = std(err_diff_vec - 1);
fprintf('-- -- -- -- -- -- -- -- -- -- -- -- -- -- --\n')
fprintf('Entre puntos deberia medir 1 metro\n')
fprintf('\tError promedio:\t%0.4f m\n\tSigma:\t\t%0.4f m\n\n', ...
  err_mu,err_sigma)
fprintf('-- -- -- -- -- -- -- -- -- -- -- -- -- -- --\n')
