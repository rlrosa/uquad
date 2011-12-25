% Add paths to all dirs with .m files

uquad_src = pwd;
if(isunix)
  slash = '/';
else
  slash = '\';
end
% GPS support scripts
addpath(sprintf('%s%cgps',uquad_src,slash));
% IMU support scripts
addpath(sprintf('%s%cimu',uquad_src,slash));

% set format to avoid confusing GPS data
format long g
