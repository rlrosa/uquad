figure
hold on
grid

colors = 'bgky';
for i = 1:4
  [easting, northing, elevation, utm_zone, sat] = ...
    gpxlogger_xml_handler(sprintf('0%d.log',i),0);
  plot(elevation,sprintf('.-%c',colors(i)))
end

hold off
title('GPS data - Atril')
xlabel('Time (s)')
ylabel('Elevation (m)')
