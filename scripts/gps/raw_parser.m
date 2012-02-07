function [easting, northing, utm] = raw_parser(filename, raw_file)

f_gpgga = sprintf('recortado.gpgga');

if(raw_file)
  printf('Asumiendo que se trata de un log crudo, generado mediante cat o gpspipe.\nRET para continuar, Ctrl+C para cancelar...')
  pause
  system(sprintf('cat %s | grep GPGGA > ./%s',filename,f_gpgga));
  fprintf('Dale con el emacs a %s. Los comandos son:\nreplace-regexp RET\n \$GPGGA,[0-9]+.[0-9]+,\([0-9]\{2\}\)\([0-9]\{2\}\).\([0-9]\{4\}\),S,\([0-9]\{3\}\)\([0-9]\{2\}\).\([0-9]\{4\}\).+$ RET\n -\1 \2 0.\3 -\4 \5 0.\6 RET\nPara continuar (dsp de que este listo) tocar RET\n',f_gpgga);
  pause
else
  f_gpgga = filename;
end

data = load(f_gpgga);
if(raw_file)
  system(sprintf('rm %s',f_gpgga));
end
lat = dms2degrees([data(:,1) data(:,2) 60*data(:,3)]);
lon = dms2degrees([data(:,4) data(:,5) 60*data(:,6)]);

[easting, northing, utm] = deg2utm(lat,lon);

%%
% %# line format for each type of message IDs
% frmt = {'$GPGGA', '%s %d.%d%d.%d,%c,%d.%d,%c %f %f %f %f %c %f %c %f %s'};
% %    '$GPGSA', '%s %c %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %s' ; 
% %    '$GPGSV', '%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %s' ; 
% %    '$GPRMC', '%s %f %c %f %c %f %c %f %f %f %f %s' ; 
% %};
% 
% len = size(f_text,1);
% 
% 
% 
% 
% 
% %# get message ID of each line
% msgId = strtok(f_text,',');
% 
% %# for each possible message ID
% arr = cell(size(frmt,1),1);
% for m=1:size(frmt,1)
%     %# get lines matching this ID
%     lines = f_text( ismember(msgId,frmt{m,1}) );
% 
%     %# parse lines using specified format
%     arr{m} = cell(numel(lines), sum(frmt{m,2}=='%'));
%     for i=1:numel(lines)
%         arr{m}(i,:) = textscan(lines{i}, frmt{m,2}, 'Delimiter',',');
%     end
% 
%     %# flatten nested cells containing strings
%     idx = cellfun(@iscell, arr{m}(1,:));
%     arr{m}(:,idx) = cellfun(@(x)x, arr{m}(:,idx));
% end
% 
% 
% 
% 
% % 
% % [lat_d, lat_m, lat_s, lon_d, lon_m, lon_s] = textread(filename, ...
% %   ''...
% %   );
% % 
% % [names,types,x,y,answer] = textread('mydata.dat','%s %s %f ...
% %     %d %s',1)
% %     returns
% %     
% fclose(fid);