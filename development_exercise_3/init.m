
%% GROUP_01 DETAILS
%  Authors:
%  - Modena Andrea 2157605
%  - Comunello Marco 2193512
%  - De Guio Riccardo 2157243
%  - Borghesan Alice 2196728

clear; close all; clc;

%% Phase 0: Cubesat data and orbital parameters
% ESAT: as written in the datasheet from Theia space

z = 0.1;                        % [m]
m = 1.185;                      % [kg]

Iz = 0.0065;                     % [kg*m^2]

% Calling s as tf object

s = tf('s');

%% 1) Rewrite the transfer functions as specified in Eq. 3

G_mt = 3.125 * 5 * 10^-4;           % [-] Gain due to torqrods
G   =  G_mt / (Iz*s^2);

%% 2) Translate the specifications (Eq. 7)
% _star => required quantity

e_rp_star = 0.04;                   % [deg] maximum steady state error
t_r_star_min = 0.8;                 % [s]   minimum rise time
t_r_star_max = 1.2;                 % [s]   maximum rise time
m_phi_star = 45;                    % [deg] minimum phase margin
t_over = 30;                        % [%]   maximum overshoot
t_set = 30;                         % [s]   maximum settling time (= simulation time)

%% 3) Calculate the static gains of the controllers K (Eq. 6)
% Trial and error

K_P = 3.1;    
K_I = 0.5;
K_D = 6;

%% 4) Verify the system stability requirements

% Not needed: the system is always BIBO stable

%% 5) Write the open-loop (C(s)G(s) and closed-loop (W(s)) transfer functions for the feedback system

% Controller (PID) definitions
C = K_P + K_I * (1/s) + K_D*s;

% Open loop transfer functions
L = C * G;

% Closed loop transfer function
W = feedback(L, 1);

%% 6) Trace the Bode diagrams of the open-loop transfer function using MATLAB bode() function
figure;
bode(L, {1e-5,1e5}); grid on; title('Bode Diagram - Yaw axis');

%% 7) Adjust the controller parameters through trial and error using MATLAB's margin() function

% NO OPTIMIZATION: just manual trial and error

figure;
margin(L, {1e-5,1e5}); grid on; title('Bode Diagram - Yaw axis');
[Gm,Pm,Wcg,Wcp]=margin(L);

%% 8) Verify the step response using MATLAB's step() function

figure;
step(W); grid on; title('Step Response - Yaw axis');
S = stepinfo(W);

% Steady state error calculation
[y,t] = step(deg2rad(45)*W);
sse = abs(45 - rad2deg(y(end)));

%% 9) Display the step response characteristics
fprintf('Phase Margin: %.2f deg\n', Pm);
fprintf('Settling Time: %.2f s\n', S.SettlingTime);
fprintf('Steady-State Error: %.2f deg\n', sse);
fprintf('Overshoot: %.2f%%\n', S.Overshoot);
fprintf('Rise Time: %.2f s\n', S.RiseTime);

%% 10) Pass-Fail Test for Requirements

% Extracting step response characteristics
rise_time = S.RiseTime;
settling_time = S.SettlingTime;
overshoot = S.Overshoot;
phase_margin = Pm;

% Pass-Fail conditions
pass = true; % Initialize pass condition

% Check phase margin
if phase_margin < m_phi_star
    fprintf('Fail: Phase Margin %.2f deg (Required: >= %.2f deg)\n', phase_margin, m_phi_star);
    pass = false;
end

% Check settling time
if settling_time > t_set
    fprintf('Fail: Settling Time %.2f s (Required: <= %.2f s)\n', settling_time, t_set);
    pass = false;
end

% Check steady state error
if sse > e_rp_star
    fprintf('Fail: Steady State Error %.2f deg (Required: <= %.2f deg)\n', sse, e_rp_star);
    pass = false;
end

% Check overshoot
if overshoot > t_over
    fprintf('Fail: Overshoot %.2f%% (Required: <= %.2f%%)\n', overshoot, t_over);
    pass = false;
end

% Check rise time
if rise_time < t_r_star_min || rise_time > t_r_star_max
    fprintf('Fail: Rise Time %.2f s (Required: %.2f - %.2f s)\n', rise_time, t_r_star_min, t_r_star_max);
    pass = false;
end

% Final result
if pass
    fprintf('Pass: All requirements satisfied.\n');
else
    fprintf('Test completed with failures.\n');
end