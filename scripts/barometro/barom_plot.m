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

figure;
% plot data
section = [avg_size:length(alt)];
hold on; 
alt_moving_avg = moving_avg(alt, avg_size);
% plot avg
hl1 = line(section, alt(section),'Color','b');
hl2 = line(section,alt_moving_avg(section),'Color','g');
legend('Datos crudos',sprintf('Promedio en %d muestras',avg_size), ...
  'Location','NorthWest');
ylabel('Altura respecto al nivel del mar (m)')
xlabel('# de muestra')
axis([1 length(alt) min(alt) max(alt)])

if(indexes ~= -1)
  % plot segment borders
  for i=1:length(indexes)
      line([indexes(i) indexes(i)],[min(alt) max(alt)],'color','black')
  end
end

ax1 = gca;
ax_plot = ax1;

if(exist('temp','var'))
  % http://www.mathworks.com/help/techdoc/creating_plots/f1-11215.html
  set(ax1,'XColor','k','YColor','k')
  ax2 = axes('Position',get(ax1,'Position'),...
             'XAxisLocation','top',...
             'YAxisLocation','right',...
             'Color','none',...
             'XColor','k','YColor','r');

  % Draw the second set of data in the same color as the x- and y-axis.
  hl3 = line(section,temp(section),'Color','r','Parent',ax2);

  set(ax1,'XColor','k','YColor','k')
  legend('Temperatura');
  ylabel('Temperatura (Celsius)')

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

% rompe el multi eje
%axis([1 length(alt) min(alt) max(alt)])

grid on
hold off
