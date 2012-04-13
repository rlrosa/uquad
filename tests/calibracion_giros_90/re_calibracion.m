[acrud,wcrud,mcrud,tcrud,bcrud,~,~,T]=mong_read('re_calibracion/log_re_calib_1',0);
[a1,w1,euler1] = mong_conv(acrud,wcrud/14.375,mcrud,0);
b1=altitud(bcrud);

[acrud,wcrud,mcrud,tcrud,bcrud,~,~,T]=mong_read('re_calibracion/log_re_calib_2',0);
[a2,w2,euler2] = mong_conv(acrud,wcrud/14.375,mcrud,0);
b2=altitud(bcrud);

[acrud,wcrud,mcrud,tcrud,bcrud,~,~,T]=mong_read('re_calibracion/log_re_calib_3',0);
[a3,w3,euler3] = mong_conv(acrud,wcrud/14.375,mcrud,0);
b3=altitud(bcrud);

[acrud,wcrud,mcrud,tcrud,bcrud,~,~,T]=mong_read('re_calibracion/log_re_calib_4',0);
[a4,w4,euler4] = mong_conv(acrud,wcrud/14.375,mcrud,0);
b4=altitud(bcrud);
