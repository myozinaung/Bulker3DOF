function tau_rudder = rudder3DOF(states,inputs)
%% Parameters
% Ship Parameters
Lpp   = 178;
rho_w = 1025;

% Propulsion Parameters
D_P  = 5.51;    % Propeller diameter "m"
w_P0 = 0.4560;    % Wake coefficient at propeller position

% Coefficients for K_T calculation
a_0 = 0.2598;
a_1 = -0.1554;
a_2 = -0.4293;

% rudder parameters
H_R      = 8.79;        % Rudder span (height) "m"
A_R      = 34.73;        % Rudder area "m^2"
t_R      = 0.297;        % Steering resistance deduction factor
a_H0     = 0.25;        % Rudder force incrase factor_0
a_H1     = 0.575;        % Rudder force incrase factor_1
x_R      = -0.5299;        % Longi coordinate of rudder position
x_H      = -0.408;        % Verti coordinate of rudder position

l_R      = -0.936;        % An experimental constant for v_R
gamma_R1 = 0.408;       % Flow straighting coefficient for rudder starboard
gamma_R2 = 0.315;       % Flow straighting coefficient for rudder port
epsilon  = 1.118;       % Ratio of wake fractions at rudder to propeller
kappa    = 0.473;       % An experimental constant

% Re-dimensionalized the ND parameters
x_R = x_R * Lpp;
x_H = x_H * Lpp;

% Rudder aspect ratio
AR_R = (H_R^2)/A_R;
% Rudder lift gradient coefficient
f_alpha = (6.13 * AR_R)/(AR_R + 2.25);

%% extract the required variables %%
u = states(4);       % Surge speed
v = states(5);       % Sway speed
r = states(6);       % Yaw rate

delta = inputs(1);
n_P = inputs(2);

%% Longitudianal inflow velocity to rudder "u_R" %%
J_P = (u*(1 - w_P0))/(n_P*D_P);

if n_P <= 0
    u_R = epsilon*u*(1 - w_P0);
else
    J_P = (u*(1 - w_P0))/(n_P*D_P);
    K_T = a_0 + a_1*J_P + a_2*(J_P^2);
    eta_uR = D_P/H_R;
    
    if K_T < 0
        u_R = epsilon*u*(1 - w_P0);
    else
        if u == 0
            u_R = epsilon*kappa*n_P*D_P*sqrt(8/pi*a_0*eta_uR);
        else
            u_R = epsilon*u*(1 - w_P0)*sqrt(eta_uR*(1+kappa*(sqrt(1+(8*K_T/(pi*J_P^2)))-1))^2+(1-eta_uR));
        end
    end
end

%% Lateral inflow velocity to rudder "v_R" %%
U = sqrt(u^2 + v^2); % Resultant speed "U"

% Hull drift angle at midship
beta = atan2(-v,u);

% Effective inflow angle to rudder in maneuvering "beta_R"
% Non-dimensionalize
if U == 0
    r_ND = 0;
else
    r_ND   = r/(U/Lpp);
end

beta_R = beta - l_R*r_ND;

%Rewrite the rudder angle within +/- 180 degrees "kakudo subroutine"
delta = rem(delta+sign(delta)*pi,2*pi)- sign(delta)*pi;

if delta > 0    % Rudder at the starboard side
    v_R = U*gamma_R1*beta_R;
else            % Rudder at the port side
    v_R = U*gamma_R2*beta_R;
end

%% Forces and Moments Calculation
% Effective inflow angle to rudder
alpha_R = delta - atan2(v_R,u_R);

% Resultant inflow velocity to rudder
U_R = sqrt(u_R^2 + v_R^2);

% Rudder normal force
F_N = 0.5*rho_w*A_R*(U_R^2)*f_alpha*sin(alpha_R); % Only for u > 0 (advancing)
a_H = a_H0 + a_H1* J_P;

% Forces & Moments
X_R = -(1-t_R)*F_N*sin(delta);          % Surge resistance force
Y_R = -(1+a_H)*F_N*cos(delta);          % Sway resistance force
N_R = -(x_R + a_H*x_H)*F_N*cos(delta);  % Yaw moment

tau_rudder = [X_R; Y_R; N_R]; % 0.005