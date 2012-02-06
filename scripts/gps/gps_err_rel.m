function [diag_err, err_rel] = gps_err_rel(fxy,diags)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [diag_err, err_rel] = gps_err_rel(fxy,diags)
%
% Con solo fxy calcula el error respecto a las diags del poligono teorico.
% Con dos argumentos, calcula respecto al segundo argumento.
%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

% Esto va a ser lo que me equivoco en las rectas.
% Ejemplo:
%   La recta 1-2 tenia que medir 6m. diag_err(1) es cuanto me equivoque en
%   algo que tenia que dar 6m. 
diag_err = sqrt(abs(fxy));

% Esto es el error relativo.
err_rel = diag_err./diags*100;

%% Analizo por separado dependiendo de largo de la recta involucrada
% Hay 600, 848, 1200 y 1341
diag_err_f_len = zeros(4,2);
% 600
tmp = diag_err([1 3 6 8 12 13 15]);
diag_err_f_len(1,:) = [mean(tmp) std(tmp,1)];
% 848
tmp = diag_err([4 7 9 11]);
diag_err_f_len(2,:) = [mean(tmp) std(tmp,1)];
% 1200
tmp = diag_err([2 14]);
diag_err_f_len(3,:) = [mean(tmp) std(tmp,1)];
% 1341
tmp = diag_err([5 10]);
diag_err_f_len(4,:) = [mean(tmp) std(tmp,1)];


%% output
fprintf('Error respecto a las medidas esperadas para las rectas (cm):\n');
fprintf('\tmean:\t%0.0f\n\tmax:\t%0.0f\n\n', ...
  mean(diag_err), ...
  max(diag_err));
