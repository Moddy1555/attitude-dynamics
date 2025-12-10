
%% GROUP_01 DETAILS
%  Authors:
%  - Modena Andrea 2157605
%  - Comunello Marco 2193512
%  - De Guio Riccardo 2157243
%  - Borghesan Alice 2196728

clear; close all; clc;

%% Simulink Parameters
t_max = 30;
dt = 0.01;
tol = 1e-5;

%% Phase 0: Cubesat data and orbital parameters
% ESAT: as written in the datasheet from Theia space

z = 0.1;                        % [m]
m = 1.185;                      % [kg]

Iz = 0.007;                     % [kg*m^2]

% Calling s as tf object

s = tf('s');

%% 1) Rewrite the transfer functions as specified in Eq. 3

G_mt = 3.125 * 5 * 10^-4;           % [-] Gain due to torqrods
G_psi   =  G_mt / (Iz*s^2);

%% 2) Translate the specifications (Eq. 7)
% _star => required quantity

k = 1;                              % [-]   system type (PID => pole at origin)
e_rp_star = 0.03;                   % [deg] steady state error
t_r_star = 1;                       % [s]   rise time
m_phi_star = 45;                    % [deg] minimum phase margin

w_A_G_star = log(10) / t_r_star;    % [rad/s] open loop tf crossover frequency

%% 3) Calculate the static gains of the controllers K (Eq. 6)

K_P_0 = 0.1;    
K_I_0 = 0.1;
K_D_0 = 0.1;    

% Initial iteration
K_I = K_I_0;
K_P = K_P_0;
K_D = K_D_0;

%% 4) Verify the system stability requirements

% Not needed: the system is always BIBO stable

%% 5) Write the open-loop (C(s)G(s) and closed-loop (W(s)) transfer functions for the feedback system

% Controller (PID) definitions
C_psi   = K_P + K_I * (1/s) + K_D*s;

% Open loop transfer functions
L_psi   = C_psi   * G_psi;

% Closed loop transfer function
W_psi   = feedback(L_psi, 1);

%% 6) Trace the Bode diagrams of the open-loop transfer function using MATLAB bode() function
figure;
bode(L_psi, {1e-5,1e5}); grid on; title('Bode Diagram - Yaw axis');

%% 7) Adjust the controller parameters through trial and error using MATLAB's margin() function

% NO OPTIMIZATION: just manual trial and error

figure;
margin(L_psi, {1e-5,1e5}); grid on; title('Bode Diagram - Yaw axis');
[Gm_psi,Pm_psi,Wcg_psi,Wcp_psi]=margin(L_psi);

%% 8) Verify the step response using MATLAB's step() function

figure;
step(W_psi); grid on; title('Step Response - Yaw axis');
S_psi = stepinfo(W_psi);
