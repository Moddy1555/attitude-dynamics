clear; close all; clc;

%  0) Satellite parameters

Ix = 0.0040;      
Iy = 0.0050;       
Iz = 0.0030;       

omega0 = 0.001;   

s = tf('s');


%  1) Transfer Functions del sistema (Eq. 3)

alpha_phi = 1/(omega0^2 * (Iz - Iy));
alpha_psi = 1/(omega0^2 * (Iy - Ix));

G_phi   = -alpha_phi * 1 / (1 - alpha_phi*Ix*s^2);
G_theta = 1 / (Iy * s^2);
G_psi   =  alpha_psi * 1 / (1 + alpha_psi*Iz*s^2);



%  2) Specifiche (Eq. 7)

tr_star = 60;                         
omega_AG_target = log(10) / tr_star;  

%  3) Calcolo guadagni statici (Eq. 6)

erp_star = 1e-2;

K_phi_min = (1/alpha_phi) * (1/erp_star - 1);
K_psi_min = (1/alpha_psi) * (1/erp_star - 1);


K_phi = abs(K_phi_min);
K_psi = abs(K_psi_min);


% Per theta K > 0 K << 1
K_theta = 1e-3;


%  4) Condizioni di stabilità

if K_phi <= 0
    warning("K_phi NON soddisfa le condizioni di stabilità!");
end
if K_theta <= 0
    warning("K_theta NON soddisfa le condizioni di stabilità!");
end
if K_psi <= 0
    warning("K_psi NON soddisfa le condizioni di stabilità!");
end


%  5) Definizione C(s)

T1_phi = 800;     T2_phi = 0.1;
T1_theta = 400;   T2_theta = 0.05;
T1_psi = 900;     T2_psi = 0.1;

C_phi   = K_phi   * (1 + T1_phi*s)   / (1 + T2_phi*s);
C_theta = K_theta * (1 + T1_theta*s) / (1 + T2_theta*s);
C_psi   = K_psi   * (1 + T1_psi*s)   / (1 + T2_psi*s);


%  6) Open-loop chat?????? 

L_phi   = C_phi   * G_phi;
L_theta = C_theta * G_theta;
L_psi   = C_psi   * G_psi;

W_phi   = feedback(L_phi, 1);
W_theta = feedback(L_theta, 1);
W_psi   = feedback(L_psi, 1);


%  7) Bode diagrams + margin
figure;
bode(L_phi); grid on; title('Bode Diagram - φ axis');

figure;
bode(L_theta); grid on; title('Bode Diagram - θ axis');

figure;
bode(L_psi); grid on; title('Bode Diagram - ψ axis');

figure;
margin(L_phi); grid on; title('Margin - φ axis');

figure;
margin(L_theta); grid on; title('Margin - θ axis');

figure;
margin(L_psi); grid on; title('Margin - ψ axis');


%  8) Step response
figure;
step(W_phi); grid on; title('Step Response - φ axis');

figure;
step(W_theta); grid on; title('Step Response - θ axis');

figure;
step(W_psi); grid on; title('Step Response - ψ axis');