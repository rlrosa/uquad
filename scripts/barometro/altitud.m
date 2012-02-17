function alt_m = altitud(press_pas)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function alt_m = altitud(press_pas)
%
% Calcular la altura (en m) a partir de la presion (en pascales).
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% Presion a nivel del mar
p0 = 1013.25e2;

alt_m = 44330*(1-(press_pas/p0).^(1/5.255));
