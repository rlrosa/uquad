close all
clear all
clc

syms x y z ax ay az vqx vqy vqz wqx wqy wqz psi phi theta T TM1 TM2 TM3 TM4 w1 w2 w3 w4 dw1 dw2 dw3 dw4 D1 D2 D3 D4 g M Ixx Iyy Izz Izzm L

f = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,w1,w2,w3,w4,dw1,dw2,dw3,dw4,TM1,TM2,TM3,TM4,D1,D2,D3,D4) [ ...    
    psi   + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
    phi   + T*( wqy*cos(psi)-wqz*sin(psi));
    theta + T*( wqz*cos(psi)/cos(phi)+wqy*sin(psi)/cos(phi));
    vqx   + T*( vqy*wqz-vqz*wqy+g*sin(phi));
    vqy   + T*( vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi));
    vqz   + T*( vqx*wqy-vqy*wqx-g*cos(phi)*cos(psi)+1/M*(TM1+TM2+TM3+TM4));
    wqx   + T*( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w1-w2+w3-w4)+L*(TM2-TM4) )/Ixx ;
    wqy   + T*( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w1-w2+w3-w4)+L*(TM3-TM1) )/Iyy;
    wqz   + T*( -Izzm*(dw1-dw2+dw3-dw4)+D1-D2+D3-D4 )/Izz ...
    ];

F=jacobian(f(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,w1,w2,w3,w4,dw1,dw2,dw3,dw4,TM1,TM2,TM3,TM4,D1,D2,D3,D4),[psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz])

h = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM1,TM2,TM3,TM4) [ ...% Devuelve [psi;phi;theta;ax;ay;az;wqx;wqy;wqz] --> misma forma que z. Sale tmb del el imu_conv
    psi ; 
    phi ; 
    theta ; 
    vqy*wqz-vqz*wqy;
    vqz*wqx-vqx*wqz;
    vqx*wqy-vqy*wqx+1/M*(TM1+TM2+TM3+TM4);
    wqx ; 
    wqy ; 
    wqz ...
    ];    

H=jacobian(h(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM1,TM2,TM3,TM4),[psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz])
