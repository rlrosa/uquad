function [norm] = norm3(v)
% -------------------------------------------------------------------------
% function [norm] = norm3(v)
%
% norm = sqrt(v(:,1).^2 + v(:,2).^2 + v(:,3).^2)
% -------------------------------------------------------------------------

sv = size(v);
if(sv(2) ~= 3)
	error('Input size should be 3 columns, X rows');
end

norm = sqrt(v(:,1).^2 + v(:,2).^2 + v(:,3).^2);
