function dxdt = Bulker3DOF(states,inputs,BF_No)

wind_wave = 0;
U_wind = Beaufort(BF_No);

%% Parameters
% Ship Paramertes
Lpp   = 178;
B     = 32.26;
d     = 11.57;
C_B   = 0.84;
x_mid = -0.0299;

% Mass and Inertia
m_x  = 0.08;
m_y  = 0.75;
J_z  = 0.903;
k_zz = 0.254;        % Radius of Gyration about Z-axis

% Constants
rho_w = 1025;


% Ship Mass calculation
m = rho_w*Lpp*B*d*C_B;

% Re-dimensionalized the ND parameters
m_x = m_x * m;
m_y = m_y * m;

I_z = (k_zz*Lpp)^2 * m;
J_z = J_z * I_z;
J_tot   = 21863.49340;

%% Assign the variables parameters to variables
% eta
psi = states(3);
psi = rem(psi+sign(psi)*pi,2*pi)- sign(psi)*pi; %Rewrite the 'psi' within +/- 180 degrees

% nu
u = states(4);
v = states(5);
r = states(6);

%% funtion call for hull, propeller, rudder, wind and waves
tau_hull   = hull3DOF(states);
tau_prop   = prop3DOF(states,inputs);
tau_rudder = rudder3DOF(states,inputs);
tau_wind   = wind3DOF(states,U_wind);
tau_wave2  = waveDrift3DOF(states,wind_wave, BF_No);

% Forces and Moments with Wind and Wave
tau_surge = tau_hull(1)+tau_prop+tau_rudder(1)+tau_wind(1)+tau_wave2(1);
tau_sway  = tau_hull(2)+tau_rudder(2)+tau_wind(2)+tau_wave2(2);
tau_yaw   = tau_hull(3)+tau_rudder(3)+tau_wind(3)+tau_wave2(3);
tau_yaw   = tau_yaw + (tau_hull(2)+tau_wind(2))*x_mid; % Correction for CG-Midship

% Forces and Moments without Wind and Wave
% tau_surge = tau_hull(1)+X_P+tau_rudder(1);
% tau_sway  = tau_hull(2)+tau_rudder(2);
% tau_yaw   = tau_hull(3)+tau_rudder(3);
% tau_yaw   = tau_yaw + tau_hull(2)*(x_mid*Lpp); % Correction for CG-Midship

%% State Equations
% Surge Dynamics
x_dot = u*cos(psi) - v*sin(psi);    % From Ship co-codinate to Earth co-ordinate
u_dot = (m*v*r + tau_surge)/(m + m_x);

% Sway Dynamics
y_dot = v*cos(psi) + u*sin(psi);    % From Ship co-codinate to Earth co-ordinate
v_dot = (-(m+m_x)*u*r+tau_sway)/(m + m_y); 

% Yaw Dynamics
psi_dot = r;
r_dot   = tau_yaw/(I_z+J_z);

%% Output
eta_dot = [x_dot; y_dot; psi_dot];
nu_dot  = [u_dot; v_dot; r_dot];
dxdt      = [eta_dot; nu_dot];