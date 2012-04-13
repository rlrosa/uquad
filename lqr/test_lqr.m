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

aux = [ 1 0 0 0 0 0 0 0 0 0 0 0 ;
        0 1 0 0 0 0 0 0 0 0 0 0 ;
        0 0 1 0 0 0 0 0 0 0 0 0 ;
        0 0 0 0 0 1 0 0 0 0 0 0 ];

A = [Ah zeros(12,4);aux zeros(4)];
B = [Bh;zeros(4,4)];
Q=diag([1 1 1 1e2 1e2 1e2 1 1 1 1 1 1 1 1 1 1]);
R=diag([1e-2 1e-2 1e-2 1e-2]);

%K_u=uquad_lqr(Ah,Bh,Q,R);
K_u=uquad_dlqr(A,B,Q,R);
K_m=lqrd(A,B,Q,R,1e-2);

%% Controlabilidad - Observabilidad - Estabilidad

ctl  = rank(ctrb(A,B)) == 16;
obs  = rank(obsv(A,eye(16))) == 16;
stab = ctl & obs;

if(stab)
    fprintf('El sistema es Controlable y Observable.\n    ==> ES ESTABLE\n');
else
    if(ctl)
        fprintf('El sistema es Controlable\n');
    else
        fprintf('El sistema NO es Controlable\n');
    end
    if(obs)
        fprintf('El sistema es Observable\n');
    else
        fprintf('El sistema NO es Controlable\n');
    end
end