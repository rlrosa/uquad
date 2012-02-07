% Defs:
%   x = -pitch
%   y = roll
%   z = -yaw

% Using only gyros, get theta_0
x_0 = sym('x_0');
y_0 = sym('y_0');
z_0 = sym('z_0');
theta_0 = [x_0;y_0;z_0];

% Unknowns - Angles in rad
x = sym('x');
y = sym('y');
z = sym('z');
theta = [x;y;z];

% Acc data
ax = sym('ax');
ay = sym('ay');
az = sym('az');
a = [ax;ay;az];

% Rotation Matrix -> 3x3:
% R = [R_1 R_2 R_3]
% We need to find a rotation matrix that will take gravity: g = [0,0,1] to
% what the acc are reading.
% acc = R*g
% R*g = [R_1*0 R_2*0 R_3*1] = R_3 -> We do not need R_1 and R_2, but they
% will be usefull for displaying the system status.
%R = [ cos(y)*cos(z) , -cos(x)*sin(z) + sin(x)*sin(y)*cos(z) , sin(x)*sin(z) + cos(x)*sin(y)*cos(z) ; ...
%      cos(y)*sin(z) , cos(x)*cos(z) + sin(x)*sin(y)*sin(z)  , -sin(x)*cos(z) + cos(x)*sin(y)*sin(z); ...
%      -sin(y)       , sin(x)*cos(y)                         , cos(x)*cos(y)];
R_1 = [cos(y)*cos(z); cos(y)*sin(z); -sin(y)];
R_2 = [-cos(x)*sin(z) + sin(x)*sin(y)*cos(z); cos(x)*cos(z) + sin(x)*sin(y)*sin(z); sin(x)*cos(y)];
R_3 = [sin(x)*sin(z) + cos(x)*sin(y)*cos(z); -sin(x)*cos(z) + cos(x)*sin(y)*sin(z); cos(x)*cos(y)];
R = [R_1, R_2, R_3];

% Call with gyro data: (x_0,y_0,z_0)
eval_R_3_0 = @(x,y,z) [ sin(x)*sin(z) + cos(x)*sin(y)*cos(z)  ; ...
                        -sin(x)*cos(z) + cos(x)*sin(y)*sin(z) ; ...
                        cos(x)*cos(y)];

% We want to solve
%   a = R*g
% Applying normalization and the fact that g = [0,0,1]:
%   a = R_3
% Taylor order 1 around the value provided by the gyro gives a linear sys.
%
%   b = a + A*theta_0 - eval_R_3_0(x_0,y_0,z_0);
%
%   A = [ diff(R_3(1),x), diff(R_3(1),y), diff(R_3(1),z) ; ...
%         diff(R_3(2),x), diff(R_3(2),y), diff(R_3(2),z) ; ...
%         diff(R_3(3),x), diff(R_3(3),y), diff(R_3(3),z) ];
% MatLab did the math:
A = [ ...
      cos(x)*sin(z) - cos(z)*sin(x)*sin(y)  , cos(x)*cos(y)*cos(z), cos(z)*sin(x) - cos(x)*sin(y)*sin(z)  ; ...
      - cos(x)*cos(z) - sin(x)*sin(y)*sin(z), cos(x)*cos(y)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y)  ; ...
      -cos(y)*sin(x)                        ,       -cos(x)*sin(y), 0                                     ];
    
 b = [ ...
 ax - sin(x_0)*sin(z_0) + x_0*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)) + z_0*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) - cos(x_0)*cos(z_0)*sin(y_0) + y_0*cos(x)*cos(y)*cos(z) ; ...
 ay + cos(z_0)*sin(x_0) - x_0*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) + z_0*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)) - cos(x_0)*sin(y_0)*sin(z_0) + y_0*cos(x)*cos(y)*sin(z) ; ...
 az - cos(x_0)*cos(y_0) - x_0*cos(y)*sin(x) - y_0*cos(x)*sin(y)                                                                                                           ];
 
% Now c or python does the eval with data.


% Finally:
%sol = inv(A)*b;

% FAILS!! wtf!