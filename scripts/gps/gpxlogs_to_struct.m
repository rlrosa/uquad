function xs = gpxlogs_to_struct(path)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function xs = gpxlogs_to_struct(path)
%
% Loads *.log files in PATH into memory.
% 
% Inputs:
%   - path: Path to dir with gpxlogger logs.
%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if(nargin < 1)
  path = pwd;
  fprintf('Loading logs from current directory...\n\n');
end

if(isunix)
    slash = '/';
else
    slash = '\';
end
path_to_load = sprintf('%s%c',path,slash);

files = dir(sprintf('%s*.log',path_to_load));

fprintf('Loading %d log files...\n',length(files));

for i=1:length(files)
  log_name = files(i).name;
  fprintf('Loading %s...\n',log_name);
  xs(i) = xmltools(sprintf('%s%c%s',path_to_load,slash,log_name));
  fprintf('%s loaded. %d files remaining...\n',log_name, length(files) - i);
end
