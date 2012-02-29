function determinar_vel%(theta0,psi0, phi0, Vx0, Vy0, Vz0)
theta0=evalin('base','theta0');
psi0=evalin('base','psi0');
phi0=evalin('base','phi0');
Vx0=evalin('base','Vx0');
Vy0=evalin('base','Vy0');
Vz0=evalin('base','Vz0');
A=[cos(phi0)*cos(theta0) cos(theta0)*sin(phi0)*sin(psi0)-cos(phi0)*sin(theta0) sin(psi0)*sin(theta0)+cos(psi0)*cos(theta0)*sin(phi0) ;
   cos(phi0)*sin(theta0) cos(theta0)*cos(psi0)+sin(psi0)*sin(phi0)*sin(theta0) cos(psi0)*sin(theta0)*sin(phi0)-sin(psi0)*cos(theta0);
   -sin(phi0) cos(phi0)*sin(psi0) cos(phi0)*cos(psi0)];

B=[Vx0;Vy0;Vz0];

X=A\B ;

assignin('base','vq10',X(1));
assignin('base','vq20',X(2));
assignin('base','vq30',X(3));