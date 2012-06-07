function respuestas_escalon(t,X);
%---------------------------------------
%Plots de step response of variable X
%---------------------------------------
Xmax    = X(end);
Xmin    = X(1);
ind     = find(X>0.9*(Xmax-Xmin)+Xmin,1,'first');
ind2    = find(X<Xmin+0.1*(Xmax-Xmin),1,'last');
rise_t  = t(ind+1)-t(ind2+1);

figure
plot(t, X);
    
xlabel('\fontsize{16}Tiempo (s)')
ylabel('\fontsize{16}Velocidad (m/s)')
hold on
plot(t,(0.9*(Xmax-Xmin)+Xmin)*ones(length(t),1),'r--')
text(t(ceil(0.5*length(t))),0.93*(Xmax-Xmin)+Xmin,'\fontsize{12}90% del valor en regimen')
plot(t(1:end),(0.1*(Xmax-Xmin)+Xmin)*ones(length(X),1),'r--')
text(t(ceil(0.5*length(t))),0.13*(Xmax-Xmin)+Xmin,'\fontsize{12}10% del valor en regimen')
line([t(ind) t(ind)],[0 Xmax],'color','green')
line([t(ind2) t(ind2)],[0 Xmax],'color','green')
text(t(10),0.5*Xmax,['\fontsize{13}t_{rise} = ',num2str(rise_t),' s']);
grid
