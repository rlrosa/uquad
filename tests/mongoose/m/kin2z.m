function z = kin2z(kin)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Convierte del formato de kalman_in.log a el formato que entienden los .m
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

a     = kin(:,5:7);
w     = kin(:,8:10);
euler = kin(:,11:13);
% t     = kin(:,14);
b     = kin(:,end);
z = [euler a w b];
