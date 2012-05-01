function [z_out] = theta_cont(z,z_hat)

if (abs(z-z_hat) >= pi)
%     [180/pi*z 180/pi*z_hat 180/pi*(z-z_hat) fix((z-z_hat-pi)/(2*pi))]
    if (z-z_hat)>0
        z = z-fix((z-z_hat+pi)/(2*pi))*2*pi;
    else
        z = z-fix((z-z_hat-pi)/(2*pi))*2*pi;
    end
end

z_out = z;