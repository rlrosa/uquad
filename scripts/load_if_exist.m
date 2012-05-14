function data = load_if_exist(path)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
% function data = load_if_exist(path)
%
% Load data from file in 'path', or return [] if file does not exist.
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

if(exist(path,'file'))
  data = load(path);
else
  data = [];
end
