function [phi,psi]=calcu_phi_psi(ax,ay,az)
%%----------------------------------------------
% Devuelve phi y psi si le pasas la aceleracion.
% Supone que estas quieto
% ----------------------------------------------


%Verifico que no haya gimbal lock
%if (abs(ax)~=9.81) Esta es la forma de hacerlo bien pero no anda siempre
%porque puede haber gimbal lock y por error en la medición no te das cuenta
if (abs(ax)<9.72)
    phi=-180/pi*asin(ax/9.81);
    psi=180/pi*atan2(ay,az);
elseif ax > 0
    phi=-90;
    psi=0;
else 
    phi=90;
    psi=0;
end


