%El sistema de referncia con el que se trabaja es un sistema
%solidario al quadcopter con coordenadas x', y', z' que son como tengo en
%un dibujo y es un embole explicar con letras.


%Se precisan tres matrices de rotación y una traslación para pasar del
%sistemas de coordenadas del mundo al del quad.

%Se tienen 6 grados de libertad que dependen del tiempo, defino las
%variables correspondientes cada grado de libertad y al tiempo.


%%%%%%%%%%% Definición de variables y constantes%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = sym('t');

%Todas las variables son función del tiempo
x = sym('x(t)');
y = sym('y(t)');
z = sym('z(t)');

theta = sym('theta(t)');
phi = sym('phi(t)');
psis = sym('psis(t)'); %esto no es un typo, es xq psi es reservada

%Masa del quad
M = sym('M');

%Largo de un brazo
L = sym('L');

%Constante gravitacional de la tierra
g = sym('g');

%Fuerzas de empuje de cada turbina
T1= sym('T1');
T2= sym('T2');
T3= sym('T3');
T4= sym('T4');

%Momento de inercia del quad
I_xxq = sym('I_xxq');
I_xyq = sym('I_xyq');
I_xzq = sym('I_xzq');
I_yxq = sym('I_yxq');
I_yyq = sym('I_yyq');
I_yzq = sym('I_yzq');
I_zxq = sym('I_zxq');
I_zyq = sym('I_zyq');
I_zzq = sym('I_zzq');

% Momento de inercia completo
%Iq=[I_xxq I_xyq I_xzq; I_yxq I_yyq I_yzq; I_zxq I_zyq I_zzq];

%Suponiendo simetrías
Iq= [I_xxq 0 0; 0 I_yyq 0; 0 0 I_zzq];

%Momento de inercia de los motores
I_xxm = sym('I_xxm');
I_xym = sym('I_xym');
I_xzm = sym('I_xzm');
I_yxm = sym('I_yxm');
I_yym = sym('I_yym');
I_yzm = sym('I_yzm');
I_zxm = sym('I_zxm');
I_zym = sym('I_zym');
I_zzm = sym('I_zzm');

%Momento de inercia completo
%Im=[I_xxm I_xym I_xzm; I_yxm I_yym I_yzm; I_zxm I_zym I_zzm];

%Suponiendo simetrías
Im= [I_xxm 0 0; 0 I_yym 0; 0 0 I_zzm];

%Rotación del quad
w=[diff(psis); diff(phi); diff(theta)];

%Velocidades angulares de los motores no se consideran los sentidos de las
%mismas
w1 = sym('w1');
w2 = sym('w2');
w3 = sym('w3');
w4 = sym('w4');


Id=[1 0 0; 0 1 0;0 0 1];



%%%%%%%%%%% Sistema de coordenadas %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Rotación según el eje z
R_theta=[cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];

%Rotación según el eje x
R_phi=[cos(phi) 0 sin(phi) ;0 1 0; -sin(phi) 0 cos(phi)];  

%Rotación según el eje x
R_psis=[1 0 0; 0 cos(psis) sin(psis) ;0 -sin(psis)  cos(psis)];  

%Matriz cambio de base de coordenadas del mundo a coordenadas del quad 
q_T_m=simple(R_psis*R_phi*R_theta);

%Matriz cambio de base de coordenadas del quad a coordenadas del mundo
m_T_q =simple((Id/(R_theta))*(Id/(R_phi))*(Id/(R_psis)));

%%%%%%%%%%%%%%%% Primera cardinal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Fuerzas presentes gravedad en la dirección -k del sistema mundo y los
%cuatro empujes de las elices en la dirección k' del sistema quad

%Vector aceleración
a=[diff(diff(x));diff(diff(x));diff(diff(x))];

%Fuerza del empuje en el sistema relativo
f= (T1+T2+T3+T4)*[0 ; 0; 1];

%Fuerza total en el sistema mundo
F=simple([0 ; 0; -g] + m_T_q*f);

%la ecuación final es a=F/M;

%%%%%%%%%%%%%%%%%% Segunda Cardinal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%dL_q/dt = M. V_G x d(r_q)/dt + M_q(ext) 
%L, momento angular, M masa total, M_q(ext) torque, 
%si tomamos q = G (centro del quad) queda:

%dL_q/dt = M_q(ext)

%L tiene dos partes.
% Rotación propia del quad

%L = (Iq+4*Im)*w + Im*(suma de los w de los motores); 
%Los momentos de inercia están en el sistema relativo

%paso w al sistema relativo
w_sist_quad=q_T_m*w;

%L escrito en el sistema relativo
L = simple(Iq+4*Im)*w_sist_quad + Im*(w1+w2+w3+w4)*[0; 0; 1]; 

%La derivada de L es la derivada de L en el sistema relativo +w*L
dL = simple(diff(L,t)+cross(w,L));

%Momentos externos valen
M_q = L.*[-T1+T3; T2-T4;0];

%esto arma un sistema de ecuaciones de 3*3. La joda es despejar diff(theta,
%t, t), diff(phi, t, t) y diff(psis, t, t)
s = simple(dL-M_q);

%Aca despejo tita dos puntos de la expresión 1
tita_dospuntos = simple(s(1)/((4*I_xxm + I_xxq)*sin(phi(t)))-diff(theta(t), t, t));
