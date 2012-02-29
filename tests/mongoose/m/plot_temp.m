function [ax1,ax2] = plot_temp(fs,aconv,t_imu,t_posta,ind,bool)

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
           
    line(t,t_imu,'Color','m','Parent',ax2,'linewidth',2);
    if bool
        hold on
        line(T*ind,t_posta,'Color','k','Parent',ax2,'linewidth',2);
        legend_handle2=legend('\fontsize{14}Temperatura ambiente','\fontsize{14}Temperatura IMU','location','NorthEast');
    else
        legend_handle2=legend('\fontsize{14}Temperatura IMU','location','NorthEast');
    end
    xlabel('\fontsize{14}Tiempo (s)')
    ylabel('\fontsize{14}Temperatura (^oC)')
    axis([0 max(t) min(min(t_posta),min(t_imu))-1 max(max(t_posta),max(t_imu))+1])
    set(legend_handle2, 'Box', 'off','position',get(legend_handle2,'position')+[-.2 -.2 0 0])
    axes(ax1);