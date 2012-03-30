function plot_c(path)

kin     = load([path 'kalman_in.log']);
x_hat_c = load([path 'x_hat.log'    ]);
wlog    = load([path 'w.log'        ]);

a     = kin(:,5:7);
w     = kin(:,8:10);
euler = kin(:,11:13);
% t     = kin(:,14);
b     = kin(:,end);
z = [euler a w b];

plot_main(x_hat_c,z)
plot_w(wlog(:,2:end))
