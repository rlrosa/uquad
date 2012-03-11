close all
clear all
clc

syms x y z ax ay az vqx vqy vqz dvqx dvqy dvqz wqx wqy wqz psi phi theta T TM1 TM2 TM3 TM4 w1 w2 w3 w4 dw1 dw2 dw3 dw4 D1 D2 D3 D4 g M Ixx Iyy Izz Izzm L

f = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM1,TM2,TM3,TM4) [ ...    
    vqx   + T*( vqy*wqz-vqz*wqy+g*sin(phi));
    vqy   + T*( vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi));
    vqz   + T*( vqx*wqy-vqy*wqx-g*cos(phi)*cos(psi)+1/M*(TM1+TM2+TM3+TM4));
    vqy*wqz-vqz*wqy+g*sin(phi);
    vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi);
    vqx*wqy-vqy*wqz-g*cos(phi)*cos(psi)+1/M*(TM1+TM2+TM3+TM4) ...
    ];


F=jacobian(f(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM1,TM2,TM3,TM4),[vqx,vqy,vqz,dvqx,dvqy,dvqz])

h = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,dvqx,dvqy,dvqz) [ ...% Devuelve [psi;phi;theta;ax;ay;az;wqx;wqy;wqz] --> misma forma que z. Sale tmb del el imu_conv
    dvqx-vqy*wqz+vqz*wqy+g*sin(phi);
    dvqy-vqz*wqx+vqx*wqz-g*cos(phi)*sin(psi);
    dvqz-vqx*wqy+vqy*wqx-g*cos(phi)*cos(psi);
    ];    

H=jacobian(h(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,dvqx,dvqy,dvqz),[vqx,vqy,vqz,dvqx,dvqy,dvqz])
