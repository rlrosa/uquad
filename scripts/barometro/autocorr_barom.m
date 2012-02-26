[alt,indexes,temp] = barom_multi_log(pwd,0,0,1,1,20);

figure
hold on
divs = [1 5 5*3 5*4]; % 5m, 2m, 20s, 15s
colrs = 'gbkr';
for i = 1:4
  alt_cut = alt(1:round(length(alt)/9/divs(i)));
  [acf,lags,bounds] = autocorr(alt_cut);
  if(i~=4)
    line_w = 1;
  else
    line_w = 2;
  end
  plot(acf,colrs(i),'LineWidth',line_w);
  % ACF
  plot([1 length(acf)], ones(1,2)*bounds(2), ...
    sprintf('%c--',colrs(i)), 'LineWidth',line_w);
  % Limites de 95% conf.
  plot([1 length(acf)], ones(1,2)*bounds(1), ...
    sprintf('%c--',colrs(i)),'HandleVisibility','off', ...
    'LineWidth',line_w);
end
legend('5 min - ACF','5 min - 95%', ...
  '2 min - ACF','2 min - 95%', ...
  '20 seg - ACF','20 seg - 95%', ...
  '15 seg - ACF','15 seg - 95%')
grid on
ylabel('Autocorrelacion de la muestra','Fontsize',16)
xlabel('Lag','Fontsize',16)
title('Funcion de autocorrelacion (ACF) de las muestras (Normalizada)', ...
  'Fontsize',16)
set(gca,'Fontsize',16)
axis tight
  