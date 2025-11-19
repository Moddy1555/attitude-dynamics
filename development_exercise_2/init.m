%% GROUP_01 DETAILS
%  Authors:
%  - Modena Andrea 2157605
%  - Comunello Marco 2193512
%  - De Guio Riccardo 2157243
%  - Borghesan Alice 2196728

clear; close all; clc;

%% Simulink Parameters
dt = 0.01;
tol = 1e-5;

%% Phase 0: Cubesat data and orbital parameters
% Cubesat nU
nX = 2;
nY = 3;
nZ = 4;

x = 0.1*nX;                     % [m]
y = 0.1*nY;                     % [m]
z = 0.1*nZ;                     % [m]
m = 0.7*(nX + nY + nZ);         % [kg]

Ix = m/12 * (z^2+y^2);          % [kg*m^2]
Iy = m/12 * (z^2+x^2);          % [kg*m^2]
Iz = m/12 * (x^2+y^2);          % [kg*m^2]

I=[Ix  0  0;
    0 Iy  0;
    0  0 Iz];   

% Orbital parameters

re = 6371;                      % [km]
r0 = re + 700;                  % [km]
mu = 398600;                    % [km^3/s^2]

T0 = 2*pi*sqrt(r0^3/mu);        % [s]

q_IC=[0; 0; 0; 1];

omega_OI = 2*pi/T0;             % [rad/s]
omegaBI_IC = [0;-omega_OI;0];

% Renaming omega_OI for convenience
omega0 = omega_OI;

% Calling s as tf object

s = tf('s');


%% 1) Rewrite the transfer functions as specified in Eq. 3

alpha_phi = 1/(omega0^2 * (Iz - Iy));
alpha_psi = 1/(omega0^2 * (Iy - Ix));

G_phi   = -alpha_phi * 1 / (1 - alpha_phi*Ix*s^2);
G_theta = 1 / (Iy * s^2);
G_psi   =  alpha_psi * 1 / (1 + alpha_psi*Iz*s^2);

%% 2) Translate the specifications (Eq. 7)
% _star => required quantity

k = 0;                              % [-]   system type (met by default)
e_rp_star = 1e-2;                   % [rad] steady state error
t_r_star = 60;                      % [s]   rise time
m_phi_star = 60;                    % [deg] minimum phase margin

w_A_G_star = log(10) / t_r_star;    % [rad/s] open loop tf crossover frequency

%% Calculate the static gains of the controllers K (Eq. 6)
K_phi_min = (1/alpha_phi) * (1/e_rp_star - 1);
K_psi_min = (1/alpha_psi) * (1/e_rp_star - 1);

% Initial iteration
K_phi = abs(K_phi_min);
K_theta = 1e-3;             % Suggested 0 < K << 1
K_psi = abs(K_psi_min);

%% 4) Verify the system stability requirements

if alpha_phi > 0 && K_phi < abs(1/alpha_phi)
    warning("K_phi < abs(1/alpha_phi)");
end
if alpha_phi < 0 && K_phi < 0
    warning("K_phi < 0");
end
if K_theta < 0
    warning("K_theta < 0");
end
if alpha_psi > 0 && K_psi < 0
    warning("K_phi < 0");
end
if alpha_psi < 0 && K_psi < abs(1/alpha_phi)
    warning("K_phi < abs(1/alpha_phi)");
end


%% 5) Write the open-loop (C(s)G(s) and closed-loop (W(s)) transfer functions for the feedback system

T1_phi = 1000;     T2_phi = 0.1;
T1_theta = 1000;   T2_theta = 0.1;
T1_psi = 1000;     T2_psi = 0.1;

% Controller (lead compensator) definitions
C_phi   = K_phi   * (1 + T1_phi*s)   / (1 + T2_phi*s);
C_theta = K_theta * (1 + T1_theta*s) / (1 + T2_theta*s);
C_psi   = K_psi   * (1 + T1_psi*s)   / (1 + T2_psi*s);

% Open loop transfer functions
L_phi   = C_phi   * G_phi;
L_theta = C_theta * G_theta; 
L_psi   = C_psi   * G_psi;

W_phi   = feedback(L_phi, 1);
W_theta = feedback(L_theta, 1);
W_psi   = feedback(L_psi, 1);


%% 6) Trace the Bode diagrams of the open-loop transfer function using MATLAB bode() function
figure;
bode(L_phi); grid on; title('Bode Diagram - Roll axis');

figure;
bode(L_theta); grid on; title('Bode Diagram - Pitch axis');

figure;
bode(L_psi); grid on; title('Bode Diagram - Yaw axis');

%% 7) Adjust the controller parameters through trial and error using MATLAB's margin() function
figure;
margin(L_phi); grid on; title('Margin - Roll axis');

figure;
margin(L_theta); grid on; title('Margin - Pitch axis');

figure;
margin(L_psi); grid on; title('Margin - Yaw axis');


%% 8) Verify the step response using MATLAB's step() function
figure;
step(W_phi); grid on; title('Step Response - Roll axis');

figure;
step(W_theta); grid on; title('Step Response - Pitch axis');

figure;
step(W_psi); grid on; title('Step Response - Yaw axis');