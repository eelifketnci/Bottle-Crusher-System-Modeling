%% KON301E - System Modeling & Simulation
% Project: Plastic Bottle Crusher System
% Parameter Initialization Script
% Run this script before running the Simulink/Simscape models.

clear; clc;

%% 1. DC Motor Parameters
Ra = 2.0;       % Armature Resistance [Ohm]
La = 0.02;      % Armature Inductance [H]
Kb = 0.1;       % Back-EMF Constant [V/(rad/s)]
Kt = 0.1;       % Torque Constant [N.m/A] (Assumed equal to Kb)
Jm = 0.002;     % Motor Rotor Inertia [kg.m^2]
Bm = 0.1;     % Motor Viscous Friction [N.m.s/rad]

%% 2. Driver and Sensor Gains
Kd = 5;         % Motor Driver Gain (Amplifier)
Ks = 1;        % Optical Sensor Gain [V/m] (Position Feedback)

%% 3. Mechanical Transmission (Rack & Pinion)
r  = 0.015;     % Pinion Radius [m]
Jd = 0.001;     % Gear Inertia [kg.m^2]

%% 4. Hydraulic System Parameters
Ps = 50e5;      % Supply Pressure [Pa] (50 Bar)
Pt = 0;         % Tank (Return) Pressure [Pa] (Atmospheric)
rho = 850;      % Hydraulic Fluid Density [kg/m^3]
beta_oil = 1e7; % Fluid Bulk Modulus (Compressibility) [Pa]

%% 5. Actuator and Valve Parameters
A  = 0.0012;    % Piston Surface Area [m^2]
M  = 10;        % Moving Mass (Piston Rod + Load) [kg]

% Valve Geometry Parameters (Required for Simscape Physical Model)
valve_max_opening = 0.005; % Maximum Valve Opening [m] (5 mm)
valve_max_area    = 5e-5;  % Maximum Valve Orifice Area [m^2]



% NOTE for Simscape:
% In the Simscape DC Motor block, set "Rotor Inertia" to (Jm + Jd)
% to account for the total inertia reflected at the motor shaft.

%% 7. Valve Coefficients (Derived for Linear Simulink Model)
% These parameters (Kx, Kp) are calculated to linearize the valve equation
% for the mathematical block diagram in Simulink.

% Calculation of Flow Gain (Kx) [m^2/s]
% Formula derived from the orifice equation: Q = Cd * Area * sqrt(2*dP/rho)
Cd = 0.7;       % Discharge Coefficient (Typical assumption)
w_gradient = valve_max_area / valve_max_opening; % Area Gradient [m]
Kx = Cd * w_gradient * sqrt(Ps/rho);

% Pressure-Flow Coefficient (Kp) [m^3/(s.Pa)]
% Represents the flow sensitivity to pressure changes (Damping/Leakage coefficient)
Kp = 1e-7;     % Linearized Flow-Pressure Coefficient

%% Display Calculated Parameters
disp('-----------------------------------------');
disp('System Parameters Loaded Successfully.');
disp(['Calculated Flow Gain (Kx): ', num2str(Kx)]);
disp(['Calculated Pressure Coeff (Kp): ', num2str(Kp)]);
disp('-----------------------------------------');