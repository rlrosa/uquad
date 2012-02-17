function alt = barom(file,do_plot)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%function alt = barom(file,do_plot)
%
% Read barom data from log file, return altitud in m.;
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if (nargin < 2)
	do_plot = 1;
end

[~,~,~,~,b] = mong_read(file,0);

alt = altitud(b);

Ts = 15e-3;
t = [1:length(b)]*Ts;

avg_size = 20;
if(do_plot)
  barom_plot(alt,20);
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
