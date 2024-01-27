function tau_prop = prop3DOF(states,inputs)
%% Parameters
% Ship Parameters
Lpp   = 178;
rho_w = 1025;

% propulsion parameters
D_P  = 5.51;    % Propeller diameter "m"
w_P0 = 0.4560;    % Wake coefficient at propeller position
t_P  = 0.2110;    % Thrust deduction factor

% Coefficients for K_T calculation
a_0 = 0.2598;
a_1 = -0.1554;
a_2 = -0.4293;

% Coefficients for K_Q (Torque Coefficient) calculation
b_0 = 0.02387;
b_1 = -0.0073;
b_2 = -0.02975;

% eta_S = 0.98;     % Shaft efficiency

% Experimental Constants representing the wake characteristic in maneuvering
% For wake fraction correction
C_1  = 1;
C_21 = 1.1;
C_22 = 1.1;

% Longitudinal coordinate of Propeller position
x_P     = -0.5;   % Non-dimensionalized parameter

%% Get the required variables
u   = states(4);
v   = states(5);
r   = states(6);

n_P = inputs(2);

%% Start Calcualtion %%
% Wkae fraction in maneuvering motion
% Non-dimensionalized Yaw Rate "r"
U    = sqrt(u^2 + v^2);    % Resultant speed
if U == 0
    r_ND = 0;
else
    r_ND = r*Lpp/U;
end

beta = atan2(-v,u); % Hull Drift angle
beta_P = beta - x_P*r_ND; % Geometric inflow angle to propeller

% if beta_P > 0 % from MMG paper ?
%     w_P = 1 - (1-w_P0)*(1+(1-exp(-C_1*abs(beta_P)))*(C_21-1));
% else
%     w_P = 1 - (1-w_P0)*(1+(1-exp(-C_1*abs(beta_P)))*(C_22-1));
% end

w_P = w_P0*(1 - (1-(cos(beta_P))^2)*(1-abs(beta_P))); % from Yasukawa's Steady Sailing Conditions (SSC) << 1984 Shallow water

J_P = (u*(1-w_P))/(n_P*D_P); % Advance ratio
K_Q = b_0 + b_1*J_P + b_2*(J_P^2); % Torque Coefficient
Q_P = K_Q*rho_w*(n_P^2)*(D_P^5); % Propeller Torque

% Propeller thrust calculation
if n_P == 0
    X_P = (1-t_P)*a_2*rho_w*D_P^2*u^2*(1-w_P)^2;
else
    J_P = (u*(1-w_P))/(n_P*D_P);
    K_T = a_0 + a_1*J_P + a_2*(J_P^2);
    if n_P > 0
        X_P =  (1-t_P)*K_T*rho_w*n_P^2*D_P^4;
    else
        X_P = -(1-t_P)*K_T*rho_w*n_P^2*D_P^4;
    end
end
% X_P = eta_S * X_P;

tau_prop = X_P;