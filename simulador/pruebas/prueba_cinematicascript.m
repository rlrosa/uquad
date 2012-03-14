G=trim_circ(1,1/5);

ti=0;
tf=20;
time=linspace(ti,tf,(tf-ti)*5);

vq1=zeros(length(time),1);vq2=zeros(length(time),1);
vq2=zeros(length(time),1);

wq1=zeros(length(time),1);wq2=zeros(length(time),1);
wq3=zeros(length(time),1);

vq1(time > -1)=G(3);vq2(time > -1)=G(4);vq3(time > -1)=G(5);

wq1(time > -1)=G(6);wq2(time > -1)=G(7);wq3(time > -1)=G(8);

            assignin('base','psi0',G(1));
            assignin('base','phi0',G(2));
            assignin('base','theta0',0);
            assignin('base','vq10',G(3));
            assignin('base','vq20',G(4));
            assignin('base','vq30',G(5));
            assignin('base','wq10',G(6));
            assignin('base','wq20',G(7));
            assignin('base','wq30',G(8));
            assignin('base','x0',5);
            assignin('base','y0',0);
            assignin('base','z0',10);
            
            [t,X]=sim('prueba_cinematica',[ti tf], [],[time',vq1,vq2,vq3,wq1,wq2,wq3]);