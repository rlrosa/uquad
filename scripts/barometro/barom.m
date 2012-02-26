function [alt,temp,ind,ax_b,ax_t,pres] = barom(file,do_plot,raw_file)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%function [alt,temp,ind,ax_b, ax_t,pres] = barom(file,do_plot,raw_file)
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

ind = [];

if(~raw_file)
  [~,~,~,temp,b,~,ind] = mong_read(file,0);
  temp=temp/10;
else
  b = load(file);
  temp = b(:,end-1)/10;
  b = b(:,end);
end

pres = b;
alt = altitud(b);

Ts = 15e-3;
t = [1:length(b)]*Ts;

avg_size = 20;
if(do_plot)
  [ax_b ax_t] = barom_plot(alt,20,ind,temp);
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
