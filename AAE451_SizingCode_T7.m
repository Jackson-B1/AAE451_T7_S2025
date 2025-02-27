%% AAE451 Sizing Code Team 7
clc; clf; clear; close all;
%% Notes
%{
Following the diagram from the sizing lecture
Using sizing for our baseline aircraft following our mission.
%}
%% Init
W_0 = 48000; % Gross weight (lb), set as baseline aircraft to start x
Ma_max = 1.6; % Max mach #, set to requirement
AR = 4.11*Ma_max^-0.622; % Aspect Ratio from raymer 4.3.1 table 4.1 for jet fighter x
TSLW0 = 0.514*Ma_max^0.141; % Thrust to weight ratio from raymer 5.2.4 thrust matching for jet fighter x
W0S = 88.3; % Weight to area ratio (lb/ft^2), taken from baseline aircraft
K_vs = 1; % K_vs = 1.04; % Variable sweep constant, remove first % if variable sweep == yes x
W_f = 0; % Fuel weight
ec = 0.7; % Oswald efficiency for fighter raymer 5.3.7 x
M_cr = 0.9; % Cruise Mach # 
E = 5; % Loiter Time (Hr)
C_D0 = 0.015; % Zero lift drag coeff for jets in raymer 5.3.7 x
C_cr = 0.8; C_lt = 0.7; % SFC from raymer table 3.3 for low bypass turbofan x
W_mi = 327; % Missile weight lb
W_gun = 275 + 300;
W_ammo = 0.58 * 500;
W_casings = 0.26 * 500;
W_avion = 100 + 10 + 100;
W_flctl = 50;
W_frctl = 50 + 450;
W_syseq = 220 + 100;
W_crew = 0; %W_crew = W_avion + W_flctl + W_frctl + W_syseq;
W_pl = W_mi + (W_ammo - W_casings); % Weight of what leaves the aircraft while flying
%W_pl = W_mi + W_gun + W_ammo;
% Optimal Conditions (50k ft as per baseline aircraft ceiling) https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104505/f-16-fighting-falcon/
rho_opt = 0.000364; % density at 50k ft slug/ft^3 https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
a_opt = 573; % Sonic velocity knots https://www.military-airshows.co.uk/speedofsound.htm
% Sea Level Conditions
rho_sl = 0.002378; % sl/ft^3 https://www2.anac.gov.br/anacpedia/ing-por/tr1611.htm
a_sl = 661; % Sonic velocity knots https://www.military-airshows.co.uk/speedofsound.htm
% 35000 ft
rho_35k = 0.000738; % Density sl/ft^3 https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
a_35k = 576; % Sonic velocity knots https://www.military-airshows.co.uk/speedofsound.htm
%% Calculation
W_0t = W_0 * 1.5;
% Looping to get the weights set
count = 0;
while ((W_0 > 1.005*W_0t)||(W_0 < 0.995*W_0t))
    count = count + 1;
    W_f = 0;
    W_0t = W_0; % Holding old W_0
    T = TSLW0 * W_0;
    A = W_0 / W0S; % Wing area ft^2
    W_e = W_0t * fwew0(W_0t,AR,TSLW0,W0S,Ma_max,K_vs); % Finding empty weight x
    % Mission for fuel
    W_fc = zeros(1,12); % Consumed Fuel
    w1w2 = zeros(1,12); % Weight ratio
    % Takeoff x
    w1w2(1) = 0.97; % Textbook value (Low end to account for warmup?)
    W_fc(1) = ffc(W_0,w1w2(1));
    W_0 = W_0 - W_fc(1);
    % Climb to opt altitude x
    v_cr = fvo(W_0,rho_opt,A,C_D0,AR,ec);
    M_cr = v_cr / a_opt;
    w1w2(2) = 1.0065 - 0.0325 * M_cr; % Raymer 6.3.6
    W_fc(2) = ffc(W_0,w1w2(2));
    W_0 = W_0 - W_fc(2);
    % Cruise 300 nm @ opt alt & vel x
    v_cr = fvo(W_0,rho_opt,A,C_D0,AR,ec);
    M_cr = v_cr / a_opt;
    q_opt = fq(rho_opt,v_cr);
    LD_opt = fLD(q_opt,C_D0,W_0,A,AR,ec);
    w1w2(3) = fBrCr(300,C_cr,v_cr,LD_opt);
    W_fc(3) = ffc(W_0,w1w2(3));
    W_0 = W_0 - W_fc(3);
    % Patrol for 4 hr loiter 35k feet x
    v_lt = 0.76 * fvo(W_0,rho_35k,A,C_D0,AR,ec); % 0.76 taken from excel example for loiter speed
    q_lt = fq(rho_35k,v_lt);
    LD_35k = fLD(q_lt,C_D0,W_0,A,AR,ec);
    w1w2(4) = fBrLo(4,C_lt,LD_35k);
    W_fc(4) = ffc(W_0,w1w2(4));
    W_0 = W_0 - W_fc(4);
    % 100 nm dash x
    v_cr = a_35k * Ma_max;
    q_cr = fq(rho_35k,v_cr);
    LD_35k = fLD(q_cr,C_D0,W_0,A,AR,ec);
    w1w2(5) = fBrCr(100,C_cr,v_cr,LD_35k);
    W_fc(5) = ffc(W_0,w1w2(5));
    W_0 = W_0 - W_fc(5);
    % One sustained 360o turn (Ps = 0) at Mach = 1.2 x
    v_cr = a_35k * 1.2;
    q_cr = fq(rho_35k,v_cr);
    LD_35k = fLD(q_cr,C_D0,W_0,A,AR,ec);
    TW = T / W_0;
    w1w2(6) = fCbt(TW,LD_35k,v_cr,1,C_cr);
    W_fc(6) = ffc(W_0,w1w2(6));
    W_0 = W_0 - W_fc(6);
    % One sustained 360o turn (Ps = 0) at Mach = 0.9 x
    v_cr = a_35k * 0.9;
    q_cr = fq(rho_35k,v_cr);
    LD_35k = fLD(q_cr,C_D0,W_0,A,AR,ec);
    TW = T / W_0;
    w1w2(7) = fCbt(TW,LD_35k,v_cr,1,C_cr);
    W_fc(7) = ffc(W_0,w1w2(7));
    W_0 = W_0 - W_fc(7);
    % Fire Missiles x
    W_0 = W_0 - W_mi;
    % Climb to opt alt x
    w1w2(8) = 0.99; % Value from example for climbing while already at speed
    W_fc(8) = ffc(W_0,w1w2(8));
    W_0 = W_0 - W_fc(8);
    % 400 nm cruise at opt x
    v_cr = fvo(W_0,rho_opt,A,C_D0,AR,ec);
    M_cr = v_cr / a_opt;
    q_opt = fq(rho_opt,v_cr);
    LD_opt = fLD(q_opt,C_D0,W_0,A,AR,ec);
    w1w2(9) = fBrCr(400,C_cr,v_cr,LD_opt);
    W_fc(9) = ffc(W_0,w1w2(9));
    W_0 = W_0 - W_fc(9);
    % Descent to Sea Level x
    w1w2(10) = 0.992; % From textbook
    W_fc(10) = ffc(W_0,w1w2(10));
    W_0 = W_0 - W_fc(10);
    % Loiter Sea Level 0.5 hr x
    v_lt = 0.76 * fvo(W_0,rho_sl,A,C_D0,AR,ec); % 0.76 taken from excel example for loiter speed
    q_lt = fq(rho_sl,v_lt);
    LD_sl = fLD(q_lt,C_D0,W_0,A,AR,ec);
    w1w2(11) = fBrLo(0.5,C_lt,LD_sl);
    W_fc(11) = ffc(W_0,w1w2(11));
    W_0 = W_0 - W_fc(11);
    % Land x
    w1w2(12) = 0.994; % From Textbook
    W_fc(12) = ffc(W_0,w1w2(12));
    W_0 = W_0 - W_fc(12);
    % Gross
    W_f = sum(W_fc);
    W_0 = W_e + W_f + W_pl + W_crew; % New gross weight
    fprintf('It: %f\n',count);
    fprintf('W_0: %f\n',W_0);
    fprintf('W_e: %f\n',W_e);
    fprintf('W_f: %f\n',W_f);
    fprintf('\n');
end
%% Functions
% Empty To Gross Weight Ratio
%{
W_0: Gross Weight (lb)
AR: Aspect Ratio
TSLW0: T_SL/W0 Ratio (imperial)
W0S: W_0/S (lb/ft^2)
Ma_max: Max Mach #
K_vs: Variable Sweep Constant
%}
function wew0 = fwew0(W_0,AR,TSLW0,W0S,Ma_max,K_vs)
    a = -0.02; b = 2.16; c1 = -0.1; c2 = 0.2; c3 = 0.04; c4 = -0.1; c5 = 0.08; % For jet fighter
    wew0 = (a+b*(W_0^c1)*(AR^c2)*(TSLW0^c3)*(W0S^c4)*(Ma_max^c5))*K_vs;
    return;
end
% Breguet Cruise
%{
R: Range (Nmi)
C: Thrust Specific Fuel Consumption
V: Velocity
LD: L/D Ratio
%}
function w2w1 = fBrCr(R,C,V,LD)
    w2w1 = exp((-R*C)/(V*LD)); return;
end
% L/D
%{
q: Dynamic pressure
CD_0: Skin drag
W: Weight
S: Surface area
AR: Aspect Ratio
ec: efficiency
%}
function LD = fLD(q,CD_0,W,S,AR,ec)
    LD = 1/(((q*CD_0)/(W/S))+((W/S)*(1/(q*pi*AR*ec)))); return;
end
% Dynamic Pressure
%{
rho: Density (slug/ft^3)
V: Velocity (kts)
%}
function q = fq(rho,V)
    q = 0.5 * rho * (V * 1.688)^2; return;
end
% Consumed Fuel
%{
%}
function fc = ffc(W,R)
    fc = W*(1-R); return;
end
% Breguet Loiter
%{
E: Endurance
C: Thrust Specific Fuel Consumption
LD: L/D Ratio
%}
function w2w1 = fBrLo(E,C,LD)
    w2w1 = exp((-E*C)/LD); return;
end
% Optimal Velocity from example
%{
W: Weight lb
rho: density slug/ft^3
A: Wing area ft^2
C_D0: zl drag ratio
AR: aspect ratio
e: oswald efficiency
vopt: optimal velocity Knots
%}
function vopt = fvo(W,rho,A,C_D0,AR,e)
    vopt = sqrt((2*W/(rho*A))*sqrt(1/(pi*AR*e*C_D0))) / 1.688; return;
end
% Combat weight ratio
%{
TW: Thrust to weight
LD: Lift to drag
V: velocity knots
x: # full turns
C: specific fuel consumption
%}
function w2w1 = fCbt(TW,LD,V,x,C)
    g = 68680.75; % Grav accel knot/hr
    n = (TW*LD);
    d = (2*pi*V*x)/(g*sqrt((n^2)-1));
    w2w1 = 1 - C * (TW) * d; return;
end