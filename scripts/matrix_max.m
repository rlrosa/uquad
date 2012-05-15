function [maxval, maxloc_row, maxloc_col] = matrix_max(A)
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
% function [maxval, maxloc_row, maxloc_col] = matrix_max(A)
%
% Finds the max value in a matrix A.
%
% Output:
%    - maxval    : Max value of A.
%    - maxloc_row: Row where max value of A is located.
%    - maxloc_col: Col where max value of A is located.
% -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
[maxval maxloc] = max(A(:));
[maxloc_row maxloc_col] = ind2sub(size(A), maxloc);