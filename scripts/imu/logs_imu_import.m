function logs_imu_import(fileToRead1,varname)
%  Imports data from the specified file
%  FILETOREAD1:  path to file to import
%  varname: string with name of variable to create

if(nargin < 2)
  warning('Wrong input arguments!');
  return;
end

% Import the file
rawData1 = importdata(fileToRead1);

% Create new var in the base workspace
assignin('base', varname, rawData1);
