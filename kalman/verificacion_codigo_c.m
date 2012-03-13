file = 'src/build/test/kalman_test/logs/salidasalidasalidax_19_47_04.log';
F    = fopen(file);
D    = textscan(F,'%s','delimiter','\t');
fclose(F);

cols = 12;

j=1;
k=1;

N    = length(D{1});
Nn   = fix(N/cols); % Cantidad de muestras

x      = str2double(D{1}(1:cols:end));
y      = str2double(D{1}(2:cols:end));
z      = str2double(D{1}(3:cols:end));
psi    = str2double(D{1}(4:cols:end));
phi    = str2double(D{1}(5:cols:end));
theta  = str2double(D{1}(6:cols:end));
vqx    = str2double(D{1}(7:cols:end));
vqy    = str2double(D{1}(8:cols:end));
vqz    = str2double(D{1}(9:cols:end));
wqx    = str2double(D{1}(10:cols:end));
wqy    = str2double(D{1}(11:cols:end));
wqz    = str2double(D{1}(12:cols:end));

figure;
subplot(221)
    plot([0;x],'b')
    hold on; grid
    plot([0;y],'r')
    plot(b,'k')
    plot([0;z],'g')
    legend('x','y','z')
    hold off    

subplot(222)
    plot([0;psi],'b')
    hold on; grid     
    plot([0;phi],'r')
    plot([0;theta],'g')
    legend('\psi','\phi','\theta')
    hold off    
    
subplot(223)
    plot([0;vqx],'b')
    hold on; grid
    plot([0;vqy],'r')
    plot([0;vqz],'g')
    legend('v_{qx}','v_{qy}','v_{qz}')
    hold off

subplot(224)
    plot([0;wqx],'b')
    hold on; grid        
    plot([0;wqy],'r')
    plot([0;wqz],'g')
    legend('w_x','w_y','w_z')
    hold off