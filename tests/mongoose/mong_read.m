function [a,w,c,t,b] = mong_read(file,plotear)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [a,w,c,t,b] = mong_read(file,plotear)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if(nargin < 2)
  plotear = 1;
end

F    = fopen(file);
D    = textscan(F,'%s','delimiter','\t');
N    = length(D{1});
cols = 14;          % Cantidad de columnas que despliega la Mongoose
Nn   = fix(N/cols); % Cantidad de muestras

a    = zeros(Nn,3); % Datos del acelerómetro
w    = zeros(Nn,3); % Datos del giróscopo
c    = zeros(Nn,3); % Datos del compas
t    = zeros(Nn,1); % Datos del termómetro
b    = zeros(Nn,1); % Datos del barómetro

a(:,1)=str2double(D{1}(3:cols:end));
a(:,2)=str2double(D{1}(4:cols:end));
a(:,3)=str2double(D{1}(5:cols:end));
w(:,1)=str2double(D{1}(6:cols:end));
w(:,2)=str2double(D{1}(7:cols:end));
w(:,3)=str2double(D{1}(8:cols:end));
c(:,1)=str2double(D{1}(9:cols:end));
c(:,2)=str2double(D{1}(10:cols:end));
c(:,3)=str2double(D{1}(11:cols:end));
t(:,1)=str2double(D{1}(12:cols:end));
b(:,1)=str2double(D{1}(13:cols:end));

if(plotear)
  figure()
    subplot(311)
    plot(a(:,1)); hold on; plot(a(:,2),'r'); plot(a(:,3),'g'); legend('a_x','a_y','a_z');
    title('Salida del acelerómetro')
    subplot(312)
    plot(w(:,1)); hold on; plot(w(:,2),'r'); plot(w(:,3),'g'); legend('w_x','w_y','w_z');
    title('Salida del giróscopo')
    subplot(313)
    plot(c(:,1)); hold on; plot(c(:,2),'r'); plot(c(:,3),'g'); legend('w_x','w_y','w_z');
    title('Salida del magnetómetro')
end
