close all
clear all
clc

% dir = 'subida';
dir = 'bajada';
% dir = 'parcial';
int = 2;

if strcmp(dir,'subida')
    if int==1
        file = 'subida_1-2012_02_23_xx_22_16_19';
        orden = 3;
    else
        file = 'subida_2-2012_02_23_xx_23_23_12';
        orden = 5;
    end
elseif strcmp(dir,'bajada')
	if int==1
        file = 'bajada_1-2012_02_23_xx_20_59_06';
        orden = 2;
    else
        file = 'bajada_2-2012_02_23_xx_22_53_30';
        orden = 4;
    end
else
    file = 'parcial-2012_02_23_xx_20_38_09';
    orden = 1;
end
        
[a,w,m,t_imu,b,fecha,ind]=mong_read(['tests/mongoose/temperaturomometro/data/' file '.log'],0);
[aconv,wconv,mconv]=mong_conv(a,w,m,0);
t_imu=t_imu/10;

F = fopen('tests/mongoose/temperaturomometro/data/README');
D = textscan(F,'%s','delimiter','\t');
fclose(F);

k=1;
for i=1:length(D{1})
    if D{1}{i}(1)=='%'
        index(k)=i;
        k=k+1;
    end
end 

t_posta = str2double(D{1}(index(orden)+1:index(orden+1)-1));

%%

fs = 50;    
T  = 1/fs;
t  = 0:T:T*(length(aconv(:,1))-1);

figure()
    ax1=gca;
    set(ax1,'Color','none','XColor','r','YColor','r')
    line(t,aconv(:,1)); hold on; grid;
    line(t,aconv(:,2),'color','r'); line(t,aconv(:,3),'color','g');
    legend_handle=legend('\fontsize{16}a_x','\fontsize{16}a_y','\fontsize{16}a_z','location','East');
    set(legend_handle, 'Box', 'off','position',get(legend_handle,'position')+[-.05 .13 0 0])
    xlabel('\fontsize{14}Tiempo (s)')
    ylabel('\fontsize{14}Aceleraciones lineales en m/(s^2)')
    axis([0 max(t) min(min(aconv))-1 max(max(aconv))+1])

    ax2 = axes('Position',get(ax1,'Position'),...
               'XAxisLocation','top',...
               'YAxisLocation','right',...
               'Color','none',...
               'XColor','k','YColor','k');
           
    line(T*ind,t_posta,'Color','k','Parent',ax2,'linewidth',2);
    hold on
    line(t,t_imu,'Color','m','Parent',ax2,'linewidth',2);
    xlabel('\fontsize{14}Tiempo (s)')
    ylabel('\fontsize{14}Temperatura (^oC)')
    axis([0 max(t) min(min(t_posta),min(t_imu))-1 max(max(t_posta),max(t_imu))+1])
    legend_handle2=legend('\fontsize{14}Temperatura ambiente','\fontsize{14}Temperatura IMU','location','NorthEast');
    set(legend_handle2, 'Box', 'off','position',get(legend_handle2,'position')+[-.2 -.2 0 0])
    axes(ax1);