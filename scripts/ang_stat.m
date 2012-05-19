function ang_stat(eul, kin, z, x_hat_c, int)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function ang_stat(kin, z, x_hat_c, int)
%
% Estimation of angle 'eul' according to different parts of the code.
%        eul:
%           - 1: psi
%           - 2: phi
%           - 3: theta
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
uquad_colors

T_kin = kin(:,1);

if(eul == 3)
	delta_deg = 180/pi*x_hat_c(1,4+eul-1);
	fprintf('\nNOTE: Displaying theta after substracting avg\n\n');
else
	delta_deg = 0;
end

figure
hold on; grid on;

% cumsum gyro
plot(T_kin(2:end), cumsum(180/pi*z(2:end,7+eul-1)).*diff(T_kin))

% euler
plot(T_kin,180/pi*z(:,eul) - delta_deg,'*-','color',green1,'markersize',2)

% x_hat
plot(T_kin,180/pi*x_hat_c(1:end,4+eul-1) - delta_deg,'color',red2,'linewidth',2)

% gyro
plot(T_kin, 180/pi*z(:,7+eul-1),'k')

% integral
plot(int(:,1),180/pi*int(:,2+eul-1*(eul~=3)),'color',orange1)

% acc
plot(T_kin,norm3(kin(:,4:6)), ...
	'linewidth',2,'color',silver1)

legend(...
	'cumsum gyro (^o)',...
	'euler(2) (^o)'   ,...
	'x\_hat(5) (^o)'  ,...
	'gyro (^o/s)'     ,...
	'int (^o/s)'      ,...
	'norm acc (m/s^2)')

switch(eul)
	case 1
		str = '\psi';
	case 2
		str = '\phi';
	case 3
		str = '\theta';
end
title(str)