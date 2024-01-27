function tau_wind = wind_force_3DOF(variables,U_wind)
%% Get the ship parameters
Loa   = 185.86;
rho_a = 1.205;

% Get the wind related ship parameters
A_T   = 585.80;     % Transverse projected area "m^2"
A_L   = 1121.22;     % Lateral projected area "m^2"

%% Coefficients calcaulted by Fujirawa Method
X0 = 0.0269;
X1 = -1.6058;
X3 = 0.6719;
X5 = 0.0825;

Y1 = 0.7142;
Y3 = -0.1219;
Y5 = 0.0237;

N1 = -0.2980;
N2 = 0.0378;
N3 = -0.0183;

% Get the variables
psi = variables(3);
u   = variables(4);
v   = variables(5);

%% Relative Wind Speed calculation
uxr = U_wind * cos(psi) + u;       % x-component of wind speed
uyr = U_wind * sin(psi) - v;       % y-component of wind speed
alpha = atan2(uyr,uxr);           % Wind angle of attack (Wind Direction)
% uxr = abs(uxr);
% uyr = abs(uyr);

U_windr = sqrt(uxr^2+uyr^2);       % Resultant relative wind speed ralative to ship

% In case of "Control" Wind speed & Direction obtained from anemometer are relative to "Ship Speed" - Not need to calculate Relative Wind speed & Direction 

%% Coefficients for each Wind forces & Moments
CX = X0 + X1*cos(alpha) + X3*cos(3*alpha) + X5*cos(5*alpha);
CY = Y1*sin(alpha) + Y3*sin(3*alpha) + Y5*sin(5*alpha);
CN = N1*sin(alpha) + N2*sin(2*alpha) + N3*sin(3*alpha);

% Calculation of Wind Forces & Moments
X_A = 0.5*rho_a*(U_windr^2) * A_T * CX;        % Wind-induced surge resistnace
Y_A = 0.5*rho_a*(U_windr^2) * A_L * CY;        % Wind-induced sway resistance
N_A = 0.5*rho_a*(U_windr^2) * A_L * Loa * CN;  % Wind-induced yaw moment

tau_wind = [X_A; Y_A; -N_A];