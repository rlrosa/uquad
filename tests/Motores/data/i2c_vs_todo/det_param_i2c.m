%% Carga de datos

close all
clear all
clc

x=load('./tests/Motores/data/i2c_vs_todo/notas_cuad.txt');

i2c   = x(:,1);                % Comando i2c
Fm    = 9.81/1000*x(:,2)/4;    % Fuerza en N-m por motor
w     = (pi*mean(x(:,3:6)'))'; % Velocidad angular promedio de los motores (rad/s)

% PLOT VELOCIDADES DE LOS 4 MOTORES

figure
    plot(i2c,x(:,4),'s-'); hold on; 
    plot(i2c,x(:,5),'r+-');
    plot(i2c,x(:,6),'g*-');
    plot(i2c,x(:,3),'k^-');
    title('\fontsize{16}Comando i^2c vs velocidad angular')
    xlabel('\fontsize{13}Comando i^2c')
    ylabel('\fontsize{13}Velocidad angular (rad/s)')
    legend('\fontsize{14}{\color{blue}D0}','\fontsize{14}{\color{red}D2}','\fontsize{14}{\color{green}D4}','\fontsize{14}D6','location','southeast')

%% Fuerza vs. velocidad angular    

wtest = 0:450;
fprintf('Fuerza vs. velocidad angular\n')
[p_quad_FW,Ftest_cuad_FW,e_cuad_FW,sigma_cuad_FW,p_cub_FW,Ftest_cub_FW,e_cub_FW,sigma_cub_FW] = curv_fit(w,Fm,wtest);

% CALIBRACION ANTERIOR
Pviejos      = [3.7646e-5  -9.0535e-4 0.0170]; % Parametros primera calibracion
Ftest_viejos = Pviejos(1)*(wtest.^2)+Pviejos(2)*wtest+Pviejos(3)*ones(size(wtest,1),1);

% PLOT FUERZA vs. VELOCIDAD ANGULAR
figure
    plot(w,Fm,'*')
    title('\fontsize{16}Velocidad angular vs. fuerza')
    xlabel('\fontsize{13}Velocidad angular (rad/s)')
    ylabel('\fontsize{13}Fuerza (N)')
    hold on
    plot(wtest,Ftest_cuad_FW,'r')
    plot(wtest,Ftest_cub_FW,'g')
    plot(wtest,Ftest_viejos,'k')
    legend('\fontsize{13}Medias experimentales','\fontsize{13}Curva modelo cuadrático',...
        '\fontsize{13}Curva modelo cúbico','\fontsize{13}Primera calibración','location','northwest')

%% i2c vs. Fuerza

i2ctest = 0:300;
fprintf('\ni2c vs. Fuerza\n')
[p_quad_FI,Ftest_cuad_FI,e_cuad_FI,sigma_cuad_FI,p_cub_FI,Ftest_cub_FI,e_cub_FI,sigma_cub_FI] = curv_fit(i2c,Fm,i2ctest);

B_lin      = [i2c ones(size(i2c,1),1)];
p_lin      = (B_lin'*B_lin)\(B_lin'*Fm);           % parametros del modelo cuadratico
Ftest_lin  = p_lin(1)*i2ctest+p_lin(2)*ones(size(i2ctest,1),1);

e_lin      = Fm-(p_lin(1)*w+p_lin(2)*ones(size(w,1),1));
e_lin_prom = mean(e_lin);
sigma_lin  = std(e_lin);
fprintf('\nModelo Lineal\n\tError: %d\n\tSigma: %d',e_lin_prom,sigma_lin)

% PLOT i2c vs. FUERZA
figure
    plot(i2c,Fm,'*')
    title('\fontsize{16}Comando i^2c vs. fuerza')
    xlabel('\fontsize{13}Comando i^2c')
    ylabel('\fontsize{13}Fuerza (N)')
    hold on
    plot(i2ctest,Ftest_lin,'k')
    plot(i2ctest,Ftest_cuad_FI,'r')
    plot(i2ctest,Ftest_cub_FI,'g')
    legend('\fontsize{13}Medias experimentales','\fontsize{13}Curva modelo lineal','\fontsize{13}Curva modelo cuadrático',...
        '\fontsize{13}Curva modelo cúbico','location','northwest')

%% i2c vs. Velocidad Angular

fprintf('\ni2c vs. Velocidad Angular\n')
[p_quad_IW,Ftest_cuad_IW,e_cuad_IW,sigma_cuad_IW,p_cub_IW,Ftest_cub_IW,e_cub_IW,sigma_cub_IW] = curv_fit(i2c,w,i2ctest);
 
% PLOT FUERZA vs. VELOCIDAD ANGULAR
figure
    plot(i2c,w,'*')
    title('\fontsize{16}Comando i^2c vs. velocidad angular')
    xlabel('\fontsize{13}Comando i^2c')
    ylabel('\fontsize{13}Velocidad angular (rad/s)')
    hold on
    plot(i2ctest,Ftest_cuad_IW,'r')
    plot(i2ctest,Ftest_cub_IW,'g')
        legend('\fontsize{13}Medias experimentales','\fontsize{13}Curva modelo cuadrático',...
        '\fontsize{13}Curva modelo cúbico','location','northwest')

%% Velocidad Angular vs. i2c 

wtest = 0:450;
fprintf('\nVelocidad Angular vs. i^2c\n')
[p_quad_IW2,Ftest_cuad_IW2,e_cuad_IW2,sigma_cuad_IW2,p_cub_IW2,Ftest_cub_IW2,e_cub_IW2,sigma_cub_IW2] = curv_fit(w,i2c,wtest);

% PLOT FUERZA vs. VELOCIDAD ANGULAR
figure
    plot(w,i2c,'k*')
    title('\fontsize{16}Comando i^2c vs. velocidad angular')
    xlabel('\fontsize{13}Velocidad angular (rad/s)')
    ylabel('\fontsize{13}Comando i^2c')
    hold on
    plot(wtest,Ftest_cuad_IW2,'r')
    plot(wtest,Ftest_cub_IW2)
    legend('\fontsize{13}Medias experimentales','\fontsize{13}Curva modelo cuadrático',...
        '\fontsize{13}Curva modelo cúbico','location','northwest')
  
B_4      = [w.^4 w.^3 w.^2 w ones(size(w,1),1)];
p_4      = (B_4'*B_4)\(B_4'*i2c);
Ftest_4  = p_4(1)*wtest.^4+p_4(2)*wtest.^3+p_4(3)*wtest.^2+p_4(4)*wtest+p_4(5)*ones(size(wtest,1),1);

e_4      = i2c-(p_4(1)*w.^4+p_4(2)*w.^3+p_4(3)*w.^2+p_4(4)*w+p_4(5)*ones(size(w,1),1));
e_4_prom = mean(e_4);
sigma_4  = std(e_4);
fprintf('\nModelo Cuatroático\n\tError: %d\n\tSigma: %d',e_4_prom,sigma_4)
fprintf('\nEl modelo cuatroático es casi idéntico al cúbico. El término en w^4 es 6 órdenes más chico que el de w^3\n')

