function plot_c(path)

kin     = load([path 'kalman_in.log']);
x_hat_c = load([path 'x_hat.log'    ]);
wlog    = load([path 'w.log'        ]);

z = kin2z(kin);

plot_main(x_hat_c,z)
plot_w(wlog(:,2:end))
