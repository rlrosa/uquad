function alt_m = altitud(press_pas,p0)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function alt_m = altitud(press_pas)
%
% Calcular la altura (en m) a partir de la presion (en pascales).
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if nargin < 2
    p0 = mean(press_pas(1:50));   
end

alt_m = 44330*(1-(press_pas/p0).^(1/5.255));
