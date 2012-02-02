% Add paths to all dirs with .m files

uquad_root = pwd;
if(isunix)
  slash = '/';
else
  slash = '\';
end
% scripts
addpath(sprintf('%s%cscripts',uquad_root,slash));
% GPS support scripts
addpath(sprintf('%s%cscripts%cgps',uquad_root,slash,slash));
% IMU support scripts
addpath(sprintf('%s%cscripts%cimu',uquad_root,slash,slash));
addpath(genpath(sprintf('%s%ctests%cIMU',uquad_root,slash,slash)));
addpath(sprintf('%s%cSniffer_I2C',uquad_root,slash));

% set format to avoid confusing GPS data
format long g
