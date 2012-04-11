Matrices=load('linealizacion');

A=Matrices.A;
B=Matrices.B;



%% Condiciones de hovering

phi=0; psis=0; theta = 0; %Theta se puede cambiar
vq1=0; vq2=0; vq3=0;
wq1=0; wq2=0; wq3=0;

w1=316; w2=316; w3=316; w4=316;

Ah=eval(A);
Bh=eval(B);

A = [Ah zeros(12);eye(12) zeros(12)];
B = [Bh;zeros(12,4)];
Q=diag([1 1 1 1e2 1e2 1e2 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]);
R=diag([1e-2 1e-2 1e-2 1e-2]);

%K_u=uquad_lqr(Ah,Bh,Q,R);
K_u=uquad_dlqr(A,B,Q,R);
K_m=lqrd(A,B,Q,R,1e-2);


