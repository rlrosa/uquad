G=trim_circ(1,1/5);

ti=0;
tf=20;
time=linspace(ti,tf,(tf-ti)*5);

assignin('base','g',9.81); %Constante gravitacional
assignin('base','L',0.29); %Largo de los brazos del quadcopter en metros
assignin('base','M',1.541); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',2.32e-2);
assignin('base','Iyy',2.32e-2);
assignin('base','Izz',4.37e-2);
assignin('base','Izzm',1.54e-5);
wq1=zeros(length(time),1);wq2=zeros(length(time),1);
wq3=zeros(length(time),1);

w1=zeros(length(time),1);w2=zeros(length(time),1);
w3=zeros(length(time),1); w4=zeros(length(time),1);


wq1(time > -1)=G(6);wq2(time > -1)=G(7);wq3(time > -1)=G(8);

w1(time > -1)=G(9); w2(time > -1)=G(10); w3(time > -1)=G(11);
w4(time > -1)=G(12);


            assignin('base','psi0',G(1));
            assignin('base','phi0',G(2));
            assignin('base','theta0',0);
            
            assignin('base','wq10',G(6));
            assignin('base','wq20',G(7));
            assignin('base','wq30',G(8));
            
            w1()
            [t,X]=sim('pruebas_actitud',[ti tf], [],[time',w1,w2,w3,w4]);