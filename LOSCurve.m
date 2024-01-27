function [psi_d, xd, yd ,del,e] = curveLOS(variables, x_Curve, y_Curve, del, inc)
% del  = 5;
posx = variables(1);
posy = variables(2);

%% Find minimum distance point on reference curve
distAll = sqrt((x_Curve-posx).^2 + (posy-y_Curve).^2);
[distMin, idxMin] = min(distAll);
idxMin = idxMin + inc; % increase some point ahead
if idxMin >= length(x_Curve) % for final WP
    idxMin = idxMin - inc;
    xd = x_Curve([idxMin-1, idxMin]);
    yd = y_Curve([idxMin-1, idxMin]);
else

    xd = x_Curve([idxMin, idxMin+1]);
    yd = y_Curve([idxMin, idxMin+1]);
end

%% Calculate Lookahead LOS psi_d
alpha_k = atan2(yd(2) - yd(1), xd(2) - xd(1)); % Angle of straight line w.r.t NED
e       = -(posx - xd(1))*sin(alpha_k) + (posy - yd(1))*cos(alpha_k); % Cross-track error
X_r     = atan2(-e,del); % Velocity-path relative angle (~ Kp: Proportional Gain)
X_d     = alpha_k + X_r;
psi_d   = X_d;
psi_d   = rem(psi_d+sign(psi_d)*pi,2*pi)- sign(psi_d)*pi; % rewrite within -pi to pi
%% if drift angle is used
% uvel  = x(4);
% vvel  = x(5);
% beta  = atan2(-vvel,uvel);
% psi_d = psi_d + beta; % should not use drift angle if it too large (severe weather)

end