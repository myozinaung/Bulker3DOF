function [psi_d, del_t, e]   = LOSStraight(states,x_WP,y_WP,del,R_turn)

% Get the variables
x = states(1);
y = states(2);
u = states(4);

persistent k;
if isempty(k)
    k = 1;
end

% Variable Radius of circle of acceptance according to turning circle
a = sqrt((x_WP(k+2)-x_WP(k))^2 + (y_WP(k+2)-y_WP(k))^2);
b = sqrt((x_WP(k+1)-x_WP(k))^2 + (y_WP(k+1)-y_WP(k))^2);
c = sqrt((x_WP(k+2)-x_WP(k+1))^2 + (y_WP(k+2)-y_WP(k+1))^2);

alpha_r = 0.5* acos((a^2-b^2-c^2)/(-2*b*c));

R_k = R_turn(1)/(tan(alpha_r));
% 
% R_k = 2*178;
% 
% % Put R_k, Circle of acceptance within the range
R_k_min = R_turn(2);
R_k_max = R_turn(3);
if R_k < R_k_min
    R_k = R_k_min;
end

if R_k > R_k_max
    R_k = R_k_max;
end

% Waypoint Switching (Select the current 2 waypoints)
% IF the ship is within circle of acceptance
if  (x_WP(k+1)-x)^2 + (y_WP(k+1)-y)^2 <= R_k^2
    k = k+1 ;  % Increase waypoint index
end

% Angle of straight line w.r.t NED
alpha_k = atan2(y_WP(k+1) - y_WP(k), x_WP(k+1) - x_WP(k));

% Cross-track error
e = -(x - x_WP(k))*sin(alpha_k) + (y - y_WP(k))*cos(alpha_k);

% Time-varying Lookahead distnace
% del_min = 150;
% del = 300;
% del_t = -(del-del_min)/(del)*abs(e) + del;
% del_t = (del-del_min)*exp(-abs(e)/300)+del_min; % This is also good
% del_t = (del-del_min)*exp(-abs(e)/((del-del_min)/del*del_min))+del_min; % current best
% del_t = (del-del_min)*exp(-abs(e)/(0.2*del_min))+del_min;
% del_t = 200*exp(0.16*u);
% del_t = 1200*exp(-0.2*Hs);
% del_t = 600;
del_t = del;

% Velocity-path relative angle
psi_Kp = atan2(-e,del_t);

X_d = alpha_k + psi_Kp;

psi_d = X_d; % Should not add drift angle, beta become very large sometime in wave
psi_d = rem(psi_d+sign(psi_d)*pi,2*pi)- sign(psi_d)*pi;