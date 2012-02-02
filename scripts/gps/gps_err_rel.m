function err_rel = gps_err_rel(fxy,diags)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function err_rel = gps_err_rel(fxy,diags)
%
% Con solo fxy calcula el error respecto a las diags del pol teorico.
% Con dos argumentos, calcula respecto al segundo argumento.
%
% Devuelve el error relativo (multiplicar por 100 para tener el error
% relativo porcentual).
%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
diag_err = sqrt(abs(fxy));

if(nargin < 2)
  fprintf('Usando rectas teoricas...\n');
  l = 600; % lado
  dc = sqrt(2*l^2); % diag corta
  dl = sqrt((2*l)^2 + l^2); % diag larga
  diags = [...
    l;2*l;l;dc;dl; ...
    l;dc;l;dc; ...
    dl;dc;...
    l;l;2*l;...
    l];
end

err_rel = diag_err./diags;
fprintf('Error (relativo) respecto a las medidas esperadas para las rectas:\n');
fprintf('\tmean:\t%0.2f%%\n\tmax:\t%0.2f%%\n\n', ...
  mean(err_rel)*100, ...
  max(err_rel)*100);
