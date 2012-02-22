function [alt, indexes] = barom_multi_log(path, ind_plots, do_plot, raw)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [alt, indexes] = barom_multi_log(path, ind_plots, do_plot, raw)
%
% Loads *.log files in PATH into memory.
% 
% Inputs:
%   - path: Path to dir.
%   - ind_plots: Do individual plots.
%   - do_plot:  Plot concatenation of all logs.
%   - raw: logs contain raw data, only numbers.
%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if(nargin < 1)
  path = pwd;
  fprintf('Loading logs from current directory...\n\n');
end

if(nargin < 2)
  ind_plots = 0;
end

if((nargin < 3) && (nargout == 0))
  if(nargout == 0)
    do_plot = 1;
  else
    do_plot = 0;
  end
end

if(~exist('raw','var'))
  raw = 0;
end

if(isunix)
    slash = '/';
else
    slash = '\';
end
path_to_load = sprintf('%s%c',path,slash);

files = dir(sprintf('%s*.log',path_to_load));

fprintf('Loading %d log files...\n',length(files));

alt = [];
temp = [];
indexes = zeros(length(files),1);

if(isempty(files))
  fprintf('no logs found!!\n\n')
  return;
end

for i=1:length(files)
  log_name = files(i).name;
  fprintf('Loading %s...\n',log_name);
  [alt_tmp,temp_tmp,~] = barom(sprintf('%s%c%s',path_to_load,slash,log_name),ind_plots,raw);
  alt  = [alt; alt_tmp];
  temp = [temp; temp_tmp];
  indexes(i) = length(alt_tmp);
  fprintf('%s loaded. %d files remaining...\n', ...
    log_name, ...
    length(files) - i);
end

indexes = cumsum(indexes);

%% Plot
if(do_plot)
  barom_plot(alt,20,indexes,temp);
end