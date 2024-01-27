function tau_hull = hull3DOF(states)
%% Parameters
% Ship Parameters
Lpp   = 178;
d     = 11.57;
A_wet = 8750;

% Constants
rho_w = 1025;
nu_w  = 1.1883e-6;

% hull coefficients 
X_0A     = -0.0410;
m_y_X_vr = 0.2018;         % (m_y + X_vr)
dX_vr    = -0.0886;

Y_v      = -0.3345;
Y_r_m_x  = 0.0271;         % (Y_r - m_x)

N_v      = -0.1475;
N_r      = -0.0547;

% Cross Flow Drag Coefficients
C_D0     = 0.7544;
C_rY     = 1.6166;
C_rN     = 0.9695;

K        = 0.37;       % Form factor

%% Re-dimensionalized the ND hull parameters
X_0A      = X_0A     * (0.5*rho_w*Lpp*d);
m_y_X_vr  = m_y_X_vr * (0.5*rho_w*Lpp^2*d);
dX_vr     = dX_vr    * (0.5*rho_w*Lpp^2*d);

Y_v       = Y_v      * (0.5*rho_w*Lpp*d);
Y_r_m_x   = Y_r_m_x  * (0.5*rho_w*Lpp^2*d);

N_v       = N_v      * (0.5*rho_w*Lpp^2*d);
N_r       = N_r      * (0.5*rho_w*Lpp^3*d);

%% Coefficients for Polynomial Resistance Model
% Linear model Poly8:
% X_H(u) = p1*u^8 + p2*u^7 + p3*u^6 + p4*u^5 + p5*u^4 + p6*u^3 + p7*u^2 + p8*u + p9
p1 =  0.011730443043118;
p2 = -0.567604370822995;
p3 =  9.611535214759623;
p4 = -50.301341350723660;
p5 = -3.595081172933300e+02;
p6 =  5.421152833906625e+03;
p7 = -1.364270995374796e+04;
p8 =  3.519390103666107e+04;
p9 = 0;

%% get the required variables
u = states(4);
v = states(5);
r = states(6);

%% Start Calculation
% Frictional resistance coefficient "C_F0"
U = sqrt(u^2 + v^2);   % Ship resultant speed
if U < 1e-6
    U = 1e-6;
end

%% Forces and Moments induced by hull
% if u > 0        % When the ship goes forward
%     Re    = (U*Lpp)/nu_w;      % Reynold number
%     C_F0  = 0.463*(log10(Re))^(-2.6);
%     X_H = -C_F0*(0.5*rho_w*A_wet*U^2)*(1+K)+(m_y_X_vr)*v*r+dX_vr*v*r*(abs(v)/U);  % Surge resistance force
% % elseif u == 0 || U == 0
% %     X_H = 0;
% else            % When the ship goes backword
%     X_H = X_0A*u*U+(m_y_X_vr)*v*r+dX_vr*v*r*(abs(v)/U);    % Surge resistance force (Beackward)
% end

%% Polynomial Model (poly model eliminates the non-smooth problem at u = 0)
X_Hu = p1*u^8 + p2*u^7 + p3*u^6 + p4*u^5 + p5*u^4 + p6*u^3 + p7*u^2 + p8*u + p9;
X_H = - X_Hu +(m_y_X_vr)*v*r+dX_vr*v*r*(abs(v)/U);

%% Simpson 1/3 rule without loop
f1 = @(x) abs(v+C_rY*r*x).*(v+C_rY*r*x);
f2 = @(x) abs(v+C_rN*r*x).*(v+C_rN*r*x).*x;

min = -Lpp/2;   % Upper limit
max = Lpp/2;   % Lower limit
n   = 1000;   % Integration interval
dh  = (max-min)/n;
xi  = min:dh:max;

% S1 & S2 are f(v,r)
s1 = dh/3*(f1(xi(1))+2*sum(f1(xi(3:2:end-2)))+4*sum(f1(xi(2:2:end)))+f1(xi(end)));
s2 = dh/3*(f2(xi(1))+2*sum(f2(xi(3:2:end-2)))+4*sum(f2(xi(2:2:end)))+f2(xi(end)));

%% Hull Sway Force and Yaw Moment (at "u = 0" Y_H & N_H are not smooth)
Y_H = Y_v*abs(u)*v +(Y_r_m_x)*u*r - 0.5*rho_w*d*C_D0*s1;  % Sway resistance force
N_H = N_v*u*v      + N_r*abs(u)*r - 0.5*rho_w*d*C_D0*s2;  % Yaw moment

tau_hull = [X_H; Y_H; N_H];