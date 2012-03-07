close all
clear all
clc

syms x y z ax ay az vqx vqy vqz wqx wqy wqz psi phi theta T TM1 TM2 TM3 TM4 w1 w2 w3 w4 dw1 dw2 dw3 dw4 D1 D2 D3 D4 g M Ixx Iyy Izz Izzm L

f = @(x,y,z,psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,ax,ay,az,w1,w2,w3,w4,dw1,dw2,dw3,dw4,TM1,TM2,TM3,TM4,D1,D2,D3,D4) [ ...    
    x     + T *(vqx*cos(phi)*cos(theta)+vqy*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vqz*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi)) ) ;
    y     + T *(vqx*cos(phi)*sin(theta)+vqy*(sin(theta)*sin(phi)*sin(psi)+cos(psi)*cos(theta))+vqz*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi)) ) ;
    z     + T *(-vqx*sin(phi)+vqy*cos(phi)*sin(psi)+vqz*cos(psi)*cos(psi));
    ...
    psi   + T*( wqx+wqz*tan(phi)*cos(psi)+wqy*tan(phi)*sin(psi));
    phi   + T*( wqy*cos(psi)-wqz*sin(psi));
    theta + T*( wqz*cos(psi)/cos(phi)+wqy*sin(psi)/cos(phi));
    ...
    -(TM1*wqx + TM2*wqx + TM3*wqx + TM4*wqx - M*ax*wqx - M*ay*wqy - M*az*wqx + M*g*wqx*sin(phi) - M*g*wqx*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz));
    -(TM1*wqz + TM2*wqz + TM3*wqz + TM4*wqz - M*ax*wqx - M*ay*wqy - M*az*wqz + M*g*wqx*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqz*(wqx - wqz));
    -(TM1*wqz + TM2*wqz + TM3*wqz + TM4*wqz - M*ax*wqz - M*ay*wqy - M*az*wqz + M*g*wqz*sin(phi) - M*g*wqz*cos(phi)*cos(psi) - M*g*wqy*cos(phi)*sin(psi))/(M*wqy*(wqx - wqz));
    ...   
    wqx   + T*( wqy*wqz*(Iyy-Izz)+wqy*Izzm*(w1-w2+w3-w4)+L*(TM2-TM4) )/Ixx ;
    wqy   + T*( wqx*wqz*(Izz-Ixx)+wqx*Izzm*(w1-w2+w3-w4)+L*(TM3-TM1) )/Iyy;
    wqz   + T*( -Izzm*(dw1-dw2+dw3-dw4)+D1-D2+D3-D4 )/Izz ...
    ];

F=jacobian(f(x,y,z,psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,ax,ay,az,w1,w2,w3,w4,dw1,dw2,dw3,dw4,TM1,TM2,TM3,TM4,D1,D2,D3,D4),[x,y,z,psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz])


h = @(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM1,TM2,TM3,TM4) [ ... % Devuelve [psi;phi;theta;ax;ay;az;wqx;wqy;wqz] --> misma forma que z. Sale tmb del el imu_conv
    psi ; 
    phi ; 
    theta ; 
    vqy*wqz-vqz*wqy+g*sin(phi);
    vqz*wqx-vqx*wqz-g*cos(phi)*sin(psi)
    vqx*wqy-vqy*wqz-g*cos(phi)*cos(psi)+1/M*(TM1+TM2+TM3+TM4);
    wqx ; 
    wqy ; 
    wqz ...
    ];

H=jacobian(h(psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz,TM1,TM2,TM3,TM4),[x,y,z,psi,phi,theta,vqx,vqy,vqz,wqx,wqy,wqz])

