pots = zeros(4,1);
pots_computado = zeros(4,1);

colors = 'bgkr';
freqs = 1./([14; 17; 23; 35] * 1e-3);
for i=1:4
  [alt,temp,ind,ax_plot] = barom(sprintf('oss%d.log',i-1),0,0);
  oss_count = 2^(i-1);
  mat = vec2mat(alt,oss_count);
  mat = mean(mat,2);
  pots(i) = std(alt);
  pots_computado(i) = std(mat);
end

fprintf('\n-- -- -- -- -- -- -- -- -- -- -- -- --\nResultados:\n')
fprintf('\tRMS Computado\tRMS muestras\n')
for i=1:length(pots)
  fprintf('Modo %d\t%0.4f\t\t%0.4f\n',i,pots_computado(i),pots(i));
end
fprintf('-- -- -- -- -- -- -- -- -- -- -- -- --\n\n')

table_latex = 1;
if(table_latex)
  specs_m = [.5 .4 .3 .25];
  for i=1:length(pots)
    fprintf( ...
      '\\multicolumn{1}{|c|}{%d}\t&\t%0.2f\t&\t%0.2f\t&\t%0.2f\\\\\n\\hline\n', ...
      i-1, ...
      pots_computado(i), ...
      pots(i), ...
      specs_m(i));
  end
end
