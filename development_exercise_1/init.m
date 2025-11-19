clc; clear
%%Initial conditions 
%Cubesat 3U x=0.1 m; y=0.1 m; z=0.3 m; m=0.7+0.7+0.7=2.25 kg
x=0.1; y=0.1; z=0.3; m=2.25;
%Calculation of moments of inertia Ix, Iy, Iz
Iz=m/12 * (x^2+y^2) ; Ix=m/12 * (z^2+y^2); Iy=Ix; %kg*m^2
I=[Ix 0 0 ; 0 Iy 0; 0 0 Iz]; %inertia matrix
%Orbital radius r0=700 km and calculation of orbital period
%T=2pi*sqrt(r^3/mu) s
re=6371; r0=re+700; %km
mu=398600; %km^3/s^2
T0=2*pi*sqrt(r0^3/mu); %Orbital period, in s
%Sinusoidal torques Tx=5e-8 * cos(t), Ty=2e-8 * sin(t), Tz=3e-8 * sin(2t)
t=0:25:(1.2*T0); %time vector, basically the unkown variable
Tx=5e-8*cos(t + pi/2); Ty=2e-8*sin(t + pi/2); Tz=3e-8*sin(2*t + pi/2);

figure(1)
subplot(3,1,1), plot(t,Tx),xlabel('t [s]'),ylabel('Tx [Nm]');
grid on; hold on;
subplot(3,1,2), plot(t,Ty),xlabel('t [s]'),ylabel('Ty [Nm]');
grid on; hold on;
subplot(3,1,3), plot(t,Tz),xlabel('t [s]'),ylabel('Tz [Nm]');
grid on; hold on;

%Nadir pointing quaternion
qIC=[0;0;0;1];

%Orbital velocity
omega_oi=2*pi/T0;
omega_bi_IC=[0;-omega_oi;0];