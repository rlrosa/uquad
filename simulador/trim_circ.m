function G =trim_circ(Vel,thetapunto)
%--------------------------------------------------------------------------
% La función recibe el modulo de la velocidad tangencial y la velocidad
% angular
% En el vector G devuelve, phi,psis,vq1,vq2,vq3,wq1,wq2,wq3 w1 w2 w3 w4
%--------------------------------------------------------------------------

assignin('base','V',Vel);
assignin('base','thetap',thetapunto)

X0=[0 0 0 Vel 0 0 0 thetapunto 330 330 330 330];

G=fsolve(@sistema,X0);