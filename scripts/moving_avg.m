function y = moving_avg(x, avg_size)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function y = moving_avg(x, avg_size)
%
% Applies a moving average filter to the input signal.
%
% Inputs:
%   x: Input signal.
%   avg_size: Number of samples to use in the moving average.
% Outputs:
%   y: Filtered signal.
%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
y = zeros(length(x) - avg_size + 1, 1);
for n= avg_size:length(x)
    y(n)=sum( x(n-(avg_size - 1):n))/avg_size ;
end
