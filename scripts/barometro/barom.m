function [alt,temp] = barom(file,do_plot,raw_file)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%function [alt,temp] = barom(file,do_plot,raw_file)
%
% Read barom data from log file, return altitud in m, temp in C
% RAW_FILE if no text in log file (A, Z removed).
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if (nargin < 2)
	do_plot = 1;
end
if (~exist('raw_file','var'))
  raw_file = 0;
end

if(~raw_file)
  [~,~,~,~,b] = mong_read(file,0);
else
  b = load(file);
  temp = b(:,end-1)/10;
  b = b(:,end);
end

alt = altitud(b);

Ts = 15e-3;
t = [1:length(b)]*Ts;

avg_size = 20;
if(do_plot)
  barom_plot(alt,20,0,temp);
end

% ac1 =	7073;
% ac2 =	-1104;
% ac3 =	-14440;
% ac4 =	32750;
% ac5 =	24521;
% ac6 =	23940;
% b1 =	5498;
% b2 =	58;
% mb =	-32768;
% mc =	-11075;
% md =	2432;
