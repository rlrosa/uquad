
file  = '/home/rrosa/bukake';
data  = load(file);

x_hat_c = data(:,2:13);
pos     = x_hat_c(:,1:3);
euler   = x_hat_c(:,4:6);
vel     = x_hat_c(:,7:9);
wang    = x_hat_c(:,10:12);

z       = [euler zeros(size(euler)) wang];
t       = data(:,1);

w     = data(:,14:17);

plot_main(x_hat,t,z,t);
plot_w([t w]);