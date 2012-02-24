function ax_plot = barom_plot(alt, avg_size, indexes, temp)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%function barom_plot(alt, avg_size, indexes)
%
% Plot altitud, raw and averaged.
% Multiple segments can be marked by providing indexing in INDEXES.
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if(nargin < 3)
  indexes = -1;
end

font_size = 16;

figure;
% plot data
section = [avg_size:length(alt)];
hold on; 
alt_moving_avg = moving_avg(alt, avg_size);
% plot avg
hl1 = line(section, alt(section),'Color','b');
hl2 = line(section,alt_moving_avg(section),'Color','g');
ylabel('Altura respecto al nivel del mar (m)','Fontsize',font_size)
xlabel('# de muestra','Fontsize',font_size)
axis([1 length(alt) min(alt) max(alt)])

if(indexes ~= -1)
  % plot segment borders
  for i=1:length(indexes)
      line([indexes(i) indexes(i)],[min(alt) max(alt)],'color','black')
  end
end

ax1 = gca;
ax_plot = ax1;
set(ax1,'XColor','k','YColor','k')
set(ax1,'FontSize',font_size);

grid on

if(exist('temp','var'))
  if(length(temp) ~= length(alt))
    fprintf('Hay %d muestras de alt y %d muestras temp...\nDeberian ser la misma cantidad!', ...
      length(alt), length(temp));
  end
  % http://www.mathworks.com/help/techdoc/creating_plots/f1-11215.html
  ax2 = axes('Position',get(ax1,'Position'),...
             'XAxisLocation','top',...
             'YAxisLocation','right',...
             'Color','none',...
             'XColor','k','YColor','r');
  set(ax2,'FontSize',font_size);

  % Draw the second set of data in the same color as the x- and y-axis.
  hl3 = line(section,temp(section),'Color','r','Parent',ax2);

  set(ax1,'XColor','k','YColor','k')
  legend('Temperatura');
  ylabel('Temperatura (Celsius)','Fontsize',font_size)

  axis(ax2,'tight')

%   %% Match grid
%   % No hace falta, y queda horrible
%   if(0)
%     % Axis 1
%     xlimits = get(ax1,'XLim');
%     ylimits = get(ax1,'YLim');
%     xinc = (xlimits(2)-xlimits(1))/5;
%     yinc = (ylimits(2)-ylimits(1))/5;
%     % Now set the tick mark locations.
%     set(ax1,'XTick',[xlimits(1):xinc:xlimits(2)],...
%             'YTick',[ylimits(1):yinc:ylimits(2)])
%     % Axis 2
%     xlimits = get(ax2,'XLim');
%     ylimits = get(ax2,'YLim');
%     xinc = (xlimits(2)-xlimits(1))/5;
%     yinc = (ylimits(2)-ylimits(1))/5;
%     % Now set the tick mark locations.
%     set(ax2,'XTick',[xlimits(1):xinc:xlimits(2)],...
%             'YTick',[ylimits(1):yinc:ylimits(2)])
%   end
end

legend(ax1,'Datos crudos',sprintf('Promedio en %d muestras',avg_size), ...
  'Location','NorthWest');
axis(ax1,'tight')

% rompe el multi eje
%axis([1 length(alt) min(alt) max(alt)])

grid on
hold off
