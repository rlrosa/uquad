% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Este script toma las medidas de las diagonales del pol rectangulo del
% experimento del GPS, y obtiene la figura que + se parece al pol mediante
% minimos cuadrados.
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% diags es del README del 01_17_2012
% % 12 - 
% 603
% % 14 - 
% 606
% % 15 - 
% 855
% % 16 - 
% 1345
% % 23 - 
% 603
% % 24 - 
% 853
% % 25 - 
% 608
% % 26 - 
% 853
% % 34 - 
% 1344
% % 35 - 
% 850
% % 36 - 
% 602
% % 45 - 
% 602
% % 56 - 
% 603

% diags en cm
diags = [...
  603;606;855;1345; ...
  603;853;608;853; ...
  1344;850;...
  602;602;...
  603];

% puntos en un caso ideal, en m
x0 = [0 6 12 0 6 12];
y0 = [0 0 0 6 6 6];
seed = [x0';y0'];

% convierto a cm
seed = seed*100;

xy = lsqnonlin(@(xy) ...
  gps_polygon(xy,diags),seed');

points = 6;
x = xy(1:points)';
y = xy(points+1:end)';
