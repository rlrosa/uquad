function [ax1,ax2] = plot_temp(fs,aconv,t_imu,t_posta,ind,bool)

green1 = [154,205,50]/256;
green2 = [34,139,34]/256;
blue1  = [0,0,128]/256;
blue2  = [0,0,255]/256;
% red1   = [184,69,57]/256;
red1   = [178,34,34]/256;
red2   = [165,136,105]/256;

T  = 1/fs;
t  = 0:T:T*(length(aconv(:,1))-1);

figure()
    ax1=gca;
    set(ax1,'Color','none','XColor','r','YColor','r')
    hold on; grid;
    line(t,aconv(:,1),'color',red1);
    line(t,aconv(:,2),'color',blue1); 
    line(t,aconv(:,3),'color',green2);
    legend_handle=legend('\fontsize{16}a_x','\fontsize{16}a_y','\fontsize{16}a_z','location','East');
    set(legend_handle, 'Box', 'off','position',get(legend_handle,'position'))%+[-.05 -.08 0 0])
    xlabel('\fontsize{16}Tiempo (s)')
    ylabel('\fontsize{16}Aceleraciones lineales en m/(s^2)')
    axis([0 max(t) min(min(aconv))-1 max(max(aconv))+1])

    ax2 = axes('Position',get(ax1,'Position'),...
               'XAxisLocation','top',...
               'YAxisLocation','right',...
               'Color','none',...
               'XColor','k','YColor','k');
           
    line(t,t_imu,'Color',red2,'Parent',ax2,'linewidth',2);
    if bool
        hold on
        %line(T*ind,t_posta,'Color','k','Parent',ax2,'linewidth',2);
        legend_handle2=legend('\fontsize{16}Temperatura ambiente','\fontsize{16}Temperatura IMU','location','NorthEast');
    else
        legend_handle2=legend('\fontsize{16}Temperatura IMU','location','NorthEast');
    end
    xlabel('\fontsize{16}Tiempo (s)')
    ylabel('\fontsize{16}Temperatura (^oC)')
%     axis([0 max(t) min(t_imu)-2 max(t_imu)+1])
    axis([0 max(t) 30 50])
    set(legend_handle2, 'Box', 'off','position',get(legend_handle2,'position')+[0 -.2 0 0])
    axes(ax1);