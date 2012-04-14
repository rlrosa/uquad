function G =trim_circ(Velhor,thetapunto)
%--------------------------------------------------------------------------
% La función recibe el modulo de la velocidad tangencial y la velocidad
% angular
% En el vector G devuelve, phi,psis,vq1,vq2,vq3,wq1,wq2,wq3 w1 w2 w3 w4
%--------------------------------------------------------------------------

R=abs(Velhor/thetapunto);
xc=evalin('base','x0')-sign(thetapunto)*R*sin(evalin('base','theta0'));
yc=evalin('base','y0')+sign(thetapunto)*R*cos(evalin('base','theta0'));
zc=evalin('base','z0');

assignin('base','xc',xc)
assignin('base','yc',yc)
assignin('base','zc',zc)

assignin('base','Vhor',Velhor);
assignin('base','thetap',thetapunto);
assignin('base','R',R);

X0=[0 0 thetapunto 316 316 316 316];

G=fsolve(@sistema,X0,optimset('MaxFunEvals',10000));

