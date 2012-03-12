function determinar_vel%(theta0,psi0, phi0, Vx0, Vy0, Vz0)
%--------------------------------------------------------------------------
%Calcula la velocidad inicial en el sistema del cuadricóptero a partir de
%las velocidades iniciales en el sistema inercial y los angulos de Euler 
%iniciales
%--------------------------------------------------------------------------


% Cargo las condiciones iniciales
theta0=evalin('base','theta0');
psi0=evalin('base','psi0');
phi0=evalin('base','phi0');
Vx0=evalin('base','Vx0');
Vy0=evalin('base','Vy0');
Vz0=evalin('base','Vz0');


%AX=B se corresponde a las tres primeras ecuaciones del modeo físico
A=[cos(phi0)*cos(theta0) cos(theta0)*sin(phi0)*sin(psi0)-cos(phi0)*sin(theta0) sin(psi0)*sin(theta0)+cos(psi0)*cos(theta0)*sin(phi0) ;
   cos(phi0)*sin(theta0) cos(theta0)*cos(psi0)+sin(psi0)*sin(phi0)*sin(theta0) cos(psi0)*sin(theta0)*sin(phi0)-sin(psi0)*cos(theta0);
   -sin(phi0) cos(phi0)*sin(psi0) cos(phi0)*cos(psi0)];

B=[Vx0;Vy0;Vz0];

X=A\B ;

assignin('base','vq10',X(1));
assignin('base','vq20',X(2));
assignin('base','vq30',X(3));