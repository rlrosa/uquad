%%Esta parte calcula la matriz de realimentacion

Matrices=load('linealizacion.mat');
A=Matrices.A;
B=Matrices.B;


%Saco todas las variables que no vamos a usar;
A(6:8,:)=[];
A(1:2,:)=[];
A(:,6:8)=[];
A(:,1:2)=[];
B(6:8,:)=[];
B(1:2,:)=[];

wq1=0; wq2=0; wq3=0;

vq1=0; vq2=0; vq3=0;

psis=0; phi=0;  

w1=298.086701308402;
w2=298.086701308402;
w3=298.086701308402;
w4=298.086701308402;


A=eval(A);
B=eval(B);
    
Q=diag([1 1e2 1e2 1 1 1 1]);%Pesos de z,psi,phi,vqz wqx,wqy,wqz
R=diag([1 1 1 1]); %Pesos de w1 w2 w3 w4;

[K,S,E]=lqr(A,B,Q,R);

save('K','K');
assignin('base','K',K);

%% Inicialización
assignin('base','g',9.81); %Constante gravitacional
assignin('base','L',0.29); %Largo de los brazos del quadcopter en metros
assignin('base','M',1.541); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',2.32e-2);
assignin('base','Iyy',2.32e-2);
assignin('base','Izz',4.37e-2);
assignin('base','Izzm',1.54e-5);
assignin('base','Ts',5e-2)


%Tiempo de simulación.
ti=0;
tf=30;

t=linspace(ti,tf,(tf-ti)*10);

%Condiciones iniciales
X0=zeros(1,12);

assignin('base','x0',X0(1));assignin('base','y0',X0(2)); 
assignin('base','z0',X0(3));

assignin('base','psi0',X0(4));assignin('base','phi0',X0(5));
assignin('base','theta0',X0(6));

assignin('base','vq10',X0(7));assignin('base','vq20',X0(8));
assignin('base','vq30',X0(9));

assignin('base','wq10',X0(10));assignin('base','wq20',X0(11));
assignin('base','wq30',X0(12));


%Este es el setpoint de altura
assignin('base','setpoint',1);

%Defino el setpoint de las velocidade angulares
  w = zeros(4,length(t));
  w(:,:) =334.279741754537;

  
%Simulo el sistema  
[t,X,Y]=sim('primer_vuelo',[ti tf],[],[t',w(1,:)',w(2,:)',w(3,:)',w(4,:)']);