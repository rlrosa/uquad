function [p_quad,Ftest_cuad,e_cuad,sigma_cuad,p_cub,Ftest_cub,e_cub,sigma_cub] = curv_fit(w,Fm,wtest)

% MODELO CUADRATICO
B_cuad     = [w.^2 w ones(size(w,1),1)];
p_quad     = (B_cuad'*B_cuad)\(B_cuad'*Fm);           % parametros del modelo cuadratico
Ftest_cuad = p_quad(1)*(wtest.^2)+p_quad(2)*wtest+p_quad(3)*ones(size(wtest,1),1);

% MODELO CUBICO
B_cub     = [w.^3 w.^2 w ones(size(w,1),1)];
p_cub     = (B_cub'*B_cub)\(B_cub'*Fm);
Ftest_cub = p_cub(1)*(wtest.^3)+p_cub(2)*(wtest.^2)+p_cub(3)*wtest+p_cub(4)*ones(size(wtest,1),1);

% ERRORES
e_cuad      = Fm-(p_quad(1)*w.^2+p_quad(2)*w+p_quad(3)*ones(size(w,1),1));
e_cuad_prom = mean(e_cuad);
sigma_cuad  = std(e_cuad);
fprintf('Modelo Cuadrático\n\tError: %d\n\tSigma: %d\n',e_cuad_prom,sigma_cuad)

e_cub       = Fm-(p_cub(1)*(w.^3)+p_cub(2)*(w.^2)+p_cub(3)*w+p_cub(4)*ones(size(wtest,1),1));
e_cub_prom  = mean(e_cub);
sigma_cub   = std(e_cub);
fprintf('Modelo Cúbico\n\tError: %d\n\tSigma: %d\n',e_cub_prom,sigma_cub)
