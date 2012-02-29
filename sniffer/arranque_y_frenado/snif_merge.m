% Arranque completo
% Se sniffea varias veces sobreponiendo la primera parte de cada sniffeada 
% con la ultima de la sniffeada anterior.
% En este script se cargan todos los archivos y se concatenan adecuadamente
% para unificar todas las sniffeadas

% F = fopen('can2.vcd');
% D1 = textscan(F,'%s','delimiter','\t');
% fclose(F);

%% Cargo archivos

F2 = fopen('can2.vcd');
D2 = textscan(F2,'%s','delimiter','\t');
fclose(F2);

F3 = fopen('can3.vcd');
D3 = textscan(F3,'%s','delimiter','\t');
fclose(F3);

F4 = fopen('can4.vcd');
D4 = textscan(F4,'%s','delimiter','\t');
fclose(F4);

F5 = fopen('can5.vcd');
D5 = textscan(F5,'%s','delimiter','\t');
fclose(F5);

F6 = fopen('can6.vcd');
D6 = textscan(F6,'%s','delimiter','\t');
fclose(F6);

%% Concateno






