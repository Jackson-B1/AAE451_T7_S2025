%% AAE 451 GROUP 7 (HDI)

close all
clear
clc



%% Defining Input Parameters

g = 9.81; % gravity (m/s^2)


%% Takeoff curve

% Defining Takeoff Parameters
S_G = 800*0.3048; % Takeoff ground run (m)
beta = 1; % Weight fraction (1 at takeoff)
% Velocity used below is V_LOT = 120 knots (F-15 V_LOT)
q = 0.5 * 1.225 * (61.73336/sqrt(2))^2; % Dynamic Pressure (N/m^2)
CL_max = 1.2; % Max CL during TO (common value for fighter jets)
CD_TO = 0.025; % Typical turboprop military trainer value
CL_TO = 0.7; % Typical turboprop military trainer value
mu = 0.04; %Ground friction constant (typical value is 0.04)

% Equation from Gudmundsson Chapter 3
syms T_loading_TO(WingLoading)
T_loading_TO = (1.21/(g*q*CL_max*S_G)) * (WingLoading) + (0.605/CL_max) * (CD_TO - mu*CL_TO) + mu;

%% Climb Curve

% Defining Climb Parameters
% 35,000 ft in a minute in less than 4.8 nautical miles
rateofclimb = (35000/1) * (0.3048) * (1/60);
V_x = (4.8/1) * (1852) * (1/60);

V_v = rateofclimb; % Vertical speed (m/s)
Vinf = sqrt((V_v^2) + (rateofclimb^2)); % Airspeed (m/s)
q = 0.5 * (1.225/3) * (Vinf)^2; % Redefine rho
CD_min = 0.022; % Minimum CD
k = 1 / (pi * 0.73 * 2.5); % assuming AR=2.5 & e=0.73 (Tradeoff study needed)

% Equation from Gudmundsson Chapter 3
syms T_Loading_Climb(WingLoading)
T_Loading_Climb = (V_v/Vinf) + (q/WingLoading)*CD_min + (k/q) * (WingLoading);

%% +7/-3 g Turn Curve

n = sqrt(1 + (7)^2); % Load factor


syms T_Loading_Turn(WingLoading)
T_Loading_Turn = q * ((CD_min / WingLoading) + k * (n/q)^2 * WingLoading);

%% Constant Altitude/Speed Cruise (P_s = 0)

    q = 0.5 * 0.3796 * (257.9421)^2; % Dynamic pressure (using F-16 cruise speed)
    CD_min = 0.022; % Minimum CD

    % T_Loading_SL = (beta/alpha) * (K_1 * (beta/q) * (WingLoading) + K_2 + (C_D0 + C_DR)/((beta/q)*WingLoading));
syms T_Loading_SL(WingLoading)
     T_Loading_SL = (q * CD_min * (1/WingLoading)) + (k * (1/q) * WingLoading);
     
%% Plotting

fplot(T_loading_TO,"--b");
hold on
fplot(T_Loading_Climb,"--r");
fplot(T_Loading_SL,"--g");
fplot(T_Loading_Turn,"--b");
legend("Takeoff","Climb","Cruise","Sustained Turn")
xlim([1 5])
ylim([150 600])