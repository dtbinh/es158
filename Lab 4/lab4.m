%% Lab 3 Solutions

clear all, close all

% State vector: [x theta x_dot theta_dot]^T

% Define theta as angle from vertical (standing pendulum)
% theta = 0 means pendulum STANDING inverted
% x is position of cart along x axis

%% Setup Parameters
g = 9.81;           % g accel [m/s^2]
mp = 0.230;         % long pendulum mass [kg]
l = 0.6413;         % length of pendulum [m]
r = l/2;            % radius to COM [m]
J = (1/3)*mp*l^2;   % inertia of pendulum rotating about 1 end [kg-m^2]
gamma = 0.0024;     % pendulum damping [N-m*s/rad]
mc = 0.38;          % mass of cart [kg]
c = 0.90;           % cart damping [N-s/m]

%% State-Space Model

M = [mc+mp mp*r; mp*r J+mp*r^2];
beta = [c 0; 0 gamma];   % damping matrix
kappa = [0 0; 0 -mp*g*r]; % stiffness matrix
S = [1; 0]; % input weighting matrix (input is force)

A = [zeros(2) eye(2); -inv(M)*kappa -inv(M)*beta]; % A,B,C,D in block matrix form

B = [0; 0; inv(M)*S];    % input matrix

C = [1 0 0 0; 0 1 0 0];  % output matrix (select pos and angle)

D = [0; 0];              % no OL feedthrough
    
OL = ss(A, B, C, D);     % open loop system

%% Compute the numerator and denominator of the plants
[b, a] = ss2tf(A,B,C,D)

b(2,:) = b(2,:) .* [1 1 1 0 0]; 

% Adjust the numerator and denominator accordingly
P1 = tf(b(1,:),a);
P2 = tf([0 b(2,1:4)],[0 a(1:4)]); % cancel the s from the numerator and denominator

%% Use the control system toolbox to design a controller for the position
controlSystemDesigner(P2)

%% Design a very simple PID controller
z1 = [-1 -1]; 
k1 = -100;
p1 = 0; 
C1 = zpk(z1,p1,k1); 

%% Use the control system toolbox to a design a controller for the angle
controlSystemDesigner(feedback(P1, -C1))

%% Stability Analysis
sys = ss(A, B, C, D);

clf 
close all

figure(1)
pzplot(sys); 

figure(2)
bode(sys); 

figure(3)
nyquist(sys); 

%% Create the simulink model
simulink

%% Observabilty and reachability 
Wr = [B A*B A^2*B A^3*B]; 
det_Wr = det(Wr); 

Wo_p = [C(1, :); C(1, :)*A; C(1, :)*A^2; C(1, :)*A^3]; 
Wo_theta = [C(2, :); C(2, :)*A; C(2, :)*A^2; C(2, :)*A^3]; 
det_p = det(Wo_p); 
det_theta = det(Wo_theta); 

% Goes from state space to transfer function
% ss2tf

% ONce I have transfer function, then have the plant

% One plant: controlSystemDesigner(P) - help controlSystemDesigner - change
% C to change the proportional control, Change root locus to change the
% gain (imaginary axis) - see in real time what the step response is... kp
% + kis / s ... 


