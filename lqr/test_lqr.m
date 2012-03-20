Matrices=load('linealizacion');

A=Matrices.A;
B=Matrices.B;



%% Condiciones de hovering

phi=0; psis=0; theta = 0; %Theta se puede cambiar
vq1=0; vq2=0; vq3=0;
wq1=0; wq2=0; wq3=0;

w1=334.28; w2=334.28; w3=334.28; w4=334.28;

Ah=eval(A);
Bh=eval(B);

Q=diag([1 1 1 1 1 1 1 1 1 1 1 1]);
R=diag([1 1 1 1]);

%K_u=uquad_lqr(Ah,Bh,Q,R);
K_u=uquad_dlqr(Ah,Bh,Q,R);
K_m=lqrd(Ah,Bh,Q,R,5e-2);


