% Add paths to all dirs with .m files

uquad_root = pwd;
if(isunix)
  slash = '/';
else
  slash = '\';
end
% IMU support scripts
addpath(genpath(sprintf('%s%ctests%cIMU',uquad_root,slash,slash)));
<<<<<<< HEAD
addpath(sprintf('%s%cSniffer_I2C',uquad_root,slash));
=======
addpath(genpath(sprintf('%s%cscripts',uquad_root,slash)));
>>>>>>> cb874682dc6e35f8babb7adba22c43666f3c632e

% set format to avoid confusing GPS data
format long g
