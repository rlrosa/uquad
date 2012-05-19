% Add paths to all dirs with .m files

uquad_root = pwd;
if(isunix)
  slash = '/';
else
  slash = '\';
end
% IMU support scripts
addpath(genpath(sprintf('%s%ctests',uquad_root,slash)));
addpath(sprintf('%s%csniffer',uquad_root,slash));
addpath(genpath(sprintf('%s%cscripts',uquad_root,slash)));
addpath(genpath(sprintf('%s%ckalman',uquad_root,slash)));
addpath(genpath(sprintf('%s%csimulador',uquad_root,slash)));
addpath(genpath(sprintf('%s%clqr',uquad_root,slash)));
addpath(genpath(sprintf('%s%ctest_control',uquad_root,slash)));

% set format to avoid confusing GPS data
format long g

%  load colors
uquad_colors
