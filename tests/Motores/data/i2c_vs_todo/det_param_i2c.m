%% Carga de datos

close all
clear all
clc

x=load('./i2c_vs_todo/notas_cuad.txt');

i2c = x(:,1);                % Comando i2c
Fm  = 9.81/1000*x(:,2)/4;    % Fuerza en N-m por motor
w   = (pi*mean(x(:,3:6)'))'; % Velocidad angular promedio de los motores (rad/s)

wtest=0:350;

%% Plots

figure
    plot(i2c,x(:,4)); hold on; 
    plot(i2c,x(:,5),'r');
    plot(i2c,x(:,6),'g');
    plot(i2c,x(:,3),'k');
    title('comando i^2c vs velocidad angular')
    xlabel('comando i^2c')
    ylabel('Velocidad angular (rad/s)')
    legend('D0','D2','D4','D6','location','southeast')

%% Fuerza vs. velocidad angular    


% MODELO CUADRATICO
B_cuad     = [w.^2 w ones(size(w,1),1)];
p_quad     = (B_cuad'*B_cuad)\(B_cuad'*Fm);           % p_quadetros modelo cuadratico
Ftest_cuad = p_quad(1)*(wtest.^2)+p_quad(2)*wtest+p_quad(3)*ones(size(wtest,1),1);

% CALIBRACION ANTERIOR
Pviejos      = [3.7646e-5  -9.0535e-4 0.0170]; % Parametros primera calibracion
Ftest_viejos = Pviejos(1)*(wtest.^2)+Pviejos(2)*wtest+Pviejos(3)*ones(size(wtest,1),1);

% MODELO CUBICO
B_cub     = [w.^3 w.^2 w ones(size(w,1),1)];
p_cub     = (B_cub'*B_cub)\(B_cub'*Fm);
Ftest_cub = p_cub(1)*(wtest.^3)+p_cub(2)*(wtest.^2)+p_cub(3)*wtest+p_cub(4)*ones(size(wtest,1),1);

% PLOT FUERZA vs. VELOCIDAD ANGULAR
figure
    plot(w,Fm,'*')
    title('Fuerza vs. velocidad angular')
    xlabel('Velocidad angular (rad/s)')
    ylabel('Fuerza (N)')
    hold on
    plot(wtest,Ftest_cuad,'r')
    plot(wtest,Ftest_viejos,'g')
    plot(wtest,Ftest_cub,'y')
    legend('Medias experimentales','Curva modelo cuadrático','Priemra calibracion','Curva modelo cúbico')

% ERRORES
e_cuad      = Fm-(p_quad(1)*w.^2+p_quad(2)*w+p_quad(3)*ones(size(w,1),1));
e_cuad_prom = mean(e_cuad);
sigma_cuad  = std(e_cuad);
sprintf('Modelo Cuadrático\n\tError: %d\n\tSigma: %d',e_cuad_prom,sigma_cuad)

e_cub       = Fm-(p_cub(1)*(w.^3)+p_cub(2)*(w.^2)+p_cub(3)*w+p_cub(4)*ones(size(wtest,1),1));
e_cub_prom  = mean(e_cub);
sigma_cub   = std(e_cub);
sprintf('Modelo Cuadrático\n\tError: %d\n\tSigma: %d',e_cub_prom,sigma_cub)
 
 %% i2c
 
 figure
    plot(i2c,w)
 
 
 