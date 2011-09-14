%El sistema de referncia con el que se trabaja es un sistema
%solidario al quadcopter con coordenadas x', y', z' que son como tengo en
%un dibujo y es un embole explicar con letras.


%Se precisan tres matrices de rotación y una traslación para pasar del
%sistemas de coordenadas del mundo al del quad.

%Se tienen 6 grados de libertad que dependen del tiempo, defino las
%variables correspondientes cada grado de libertad y al tiempo.


%%%%%%%%%%% Definición de variables y constantes%%%%%%%%%%%%%%%%%%%%%%%%%%%
%t = sym('t');
syms t;

%Todas las variables son función del tiempo
x = sym('x(t)');
y = sym('y(t)');
z = sym('z(t)');

dx = sym('dx(t)');
dy = sym('dy(t)');
dz = sym('dz(t)');

syms ddx ddy ddz;

theta = sym('theta(t)');
phi = sym('phi(t)');
psis = sym('psis(t)'); %esto no es un typo, es xq psi es reservada

dtheta = sym('dtheta(t)');
dphi = sym('dphi(t)');
dpsis = sym('dpsis(t)');

syms ddtheta ddphi ddpsis;

%Masa del quad
M = sym('M');

%Largo de un brazo
l = sym('l');

%Constante gravitacional de la tierra
g = 9.8;

%Fuerzas de empuje de cada turbina
T1= sym('T1');
T2= sym('T2');
T3= sym('T3');
T4= sym('T4');


%Momento de inercia del quad
syms I_xx I_yy I_zz I_xxmx I_yymx I_zzmx I_xxmy I_yymy I_zzmy

I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];
Imx = [I_xxmx 0 0; 0 I_yymx 0; 0 0 I_zzmx];
Imy = [I_xxmy 0 0; 0 I_yymy 0; 0 0 I_zzmy];

%Velocidades angulares de los motores no se consideran los sentidos de las
%mismas
w1 = sym('w1(t)');
w2 = sym('w2(t)');
w3 = sym('w3(t)');
w4 = sym('w4(t)');

syms dw1 dw2 dw3 dw4;
Id=[1 0 0; 0 1 0;0 0 1];



%%%%%%%%%%% Sistema de coordenadas %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Rotación según el eje z
R_theta=[cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];

%Rotación según el eje y
R_phi=[cos(phi) 0 -sin(phi) ;0 1 0; sin(phi) 0 cos(phi)];

%Rotación según el eje x
R_psis=[1 0 0; 0 cos(psis) sin(psis) ;0 -sin(psis) cos(psis)];

%Matriz cambio de base de coordenadas del mundo a coordenadas del quad
q_T_m=simple(R_psis*R_phi*R_theta);

%Matriz cambio de base de coordenadas del quad a coordenadas del mundo
m_T_q =simple((Id/(R_theta))*(Id/(R_phi))*(Id/(R_psis)));


%Rotación del quad en el sistema prima
%w_sist_quad= [0; 0; dpsis] + R_phi*[0; dphi; 0] + R_phi*R_theta*[0; 0; dtheta];


wq1 = sym('wq1(t)');
wq2 = sym('wq2(t)');
wq3 = sym ('wq3(t)');
w_sist_quad=[wq1;wq2;wq3];


%%%%%%%%%%%%% Consideraciones Cinemáticas %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Vector Velocidad
v=[dx;dy;dz];

%Vector velocidad expresado en el sistema del quad

syms vq1 vq2 vq3 dx dy dz

vq = [vq1;vq2;vq3];

equ4 = [vq1;vq2;vq3]- R_psis*R_phi*R_theta*[dx; dy; dz];

S4 =solve(equ4(1),equ4(2),equ4(3),dx,dy,dz);
S4.dx=simple(S4.dx);
S4.dy=simple(S4.dy);
S4.dz=simple(S4.dz);


%%%%%%%%%%%%%%%% Primera cardinal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Fuerzas presentes gravedad en la dirección -k del sistema mundo y los
%cuatro empujes de las elices en la dirección k' del sistema quad


%Vector aceleración

syms dvq1 dvq2 dvq3

a=[dvq1;dvq2;dvq3] + cross(w_sist_quad,[vq1;vq2;vq3]);

%Fuerza total en el sistema quad
F=simple(q_T_m*[0 ; 0; -g]+(T1+T2+T3+T4)*[0 ; 0; 1] );

%la ecuación final es a=F/M;

caard1 = a-F/M;

%%%%%%%%%%%%%%%%%% Segunda Cardinal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%dL_q/dt = M. V_G x d(r_q)/dt + M_q(ext)
%L, momento angular, M masa total, M_q(ext) torque,
%si tomamos q = G (centro del quad) queda:

%dL_q/dt = M_q(ext)

%L tiene dos partes.
% Rotación propia del quad

%L = (Iq+4*Im)*w + Im*(suma de los w de los motores);
%Los momentos de inercia están en el sistema relativo

%L escrito en el sistema relativo
L = simple(I)*w_sist_quad + (Imx*(w1+w3)-Imy*(w2+w4))*[0; 0; 1];

%La derivada de L es la derivada de L en el sistema relativo +w*L
dL = simple(diff(L,t)+cross(w_sist_quad,L));

syms dwq1 dwq2 dwq3
dL = subs(dL,diff(wq1),dwq1);
dL = subs(dL,diff(wq2),dwq2);
dL = subs(dL,diff(wq3),dwq3);



%Momentos externos valen
M_q = l*[T2-T4; T3-T1;0];

%esto arma un sistema de ecuaciones de 3*3. La joda es despejar diff(theta,
%t, t), diff(phi, t, t) y diff(psis, t, t)
caard2 = simple(dL-M_q);



%Resulevo las ecuaciones que tengo

S1=solve(caard1(1),caard1(2),caard1(3),dvq1,dvq2,dvq3);

S1.dvq1 =simple(S1.dvq1);
S1.dvq2 =simple(S1.dvq2);
S1.dvq3 =simple(S1.dvq3);

S2=solve(caard2(1),caard2(2),caard2(3),dwq1,dwq2,dwq3);

S2.dwq1 =simple(S2.dwq1);
S2.dwq2 =simple(S2.dwq2);
S2.dwq3 =simple(S2.dwq3);

%Relación entre velocidades angulares y angulos de euler

syms wqu1 wqu2 wqu3 dphi dpsis dtheta

equ3= [wqu1;wqu2;wqu3]- ([dpsis; 0; 0] + R_psis*[0; dphi; 0] + R_psis*R_phi*[0; 0; dtheta]);

S3 = solve(equ3(1),equ3(2),equ3(3),dtheta,dphi,dpsis);

S3.dtheta =simple(S3.dtheta);
S3.dphi =simple(S3.dphi);
S3.dpsis =simple(S3.dpsis);





 



