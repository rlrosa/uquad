function [a,w,c,t,b,fecha,ind,T] = mong_read(file,plotear,imu_raw)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [a,w,c,t,b,fecha,ind,T] = mong_read(file,plotear)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if(nargin < 2)
  plotear = 1;
end

cols = 14;          % Cantidad de columnas que despliega la Mongoose

fecha = [];
ind = [];

if(exist('imu_raw','var'))
    % El archivo no tiene letras ni fechas, solo numeros.
    % Se carga mucho mas rapido asi.
    Daux = load(file);
    if(size(Daux,2) ~= 14)
        error('Log tiene # de col incorrecto!');
    end
    
    T = Daux(:,1);
    if(mean(T(1:20,1)) > 50)
        % logs con delta tiempos
        T = Daux(:,3);
        T = cumsum(T) * 1e-6; % Paso de microsegundos a segundos
    end
    a = Daux(:,4:6);
    w = Daux(:,7:9);
    c = Daux(:,10:12);
    t = Daux(:,13);
    b = Daux(:,14);
else
    F    = fopen(file);
    Daux = textscan(F,'%s','delimiter','\t');
    fclose(F);
    j=1;
    k=1;
    for i=1:length(Daux{1})
        if Daux{1}{i}(1)~='%'
            D{1}{j,1}=Daux{1}{i};
            j=j+1;
        else
            fecha{k}=Daux{1}{i};
            ind(k)=ceil(i/cols);
            k=k+1;
        end
    end

    N    = length(D{1});
    Nn   = fix(N/cols); % Cantidad de muestras

    T    = zeros(Nn,1); % Período de muestreo
    a    = zeros(Nn,3); % Datos del acelerómetro
    w    = zeros(Nn,3); % Datos del giróscopo
    c    = zeros(Nn,3); % Datos del compas
    t    = zeros(Nn,1); % Datos del termómetro
    b    = zeros(Nn,1); % Datos del barómetro

    T(:,1)=str2double(D{1}(2:cols:end));
    T = T * 1e-6; % Paso de microsegundos a segundos
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
end

if(plotear)
  figure()
    subplot(311)
    plot(a(:,1)); hold on; plot(a(:,2),'r'); plot(a(:,3),'g'); legend('\fontsize{14}a_x','\fontsize{14}a_y','\fontsize{14}a_z');
    title('\fontsize{16}Salida del acelerómetro')
    subplot(312)
    plot(w(:,1)); hold on; plot(w(:,2),'r'); plot(w(:,3),'g'); legend('\fontsize{14}w_x','\fontsize{14}w_y','\fontsize{14}w_z');
    title('\fontsize{16}Salida del giróscopo')
    subplot(313)
    plot(c(:,1)); hold on; plot(c(:,2),'r'); plot(c(:,3),'g'); legend('\fontsize{14}w_x','\fontsize{14}w_y','\fontsize{14}w_z');
    title('\fontsize{16}Salida del magnetómetro')
end
