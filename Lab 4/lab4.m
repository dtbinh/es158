%% Define the system parameters A, B, C, and D

% Constants
g = 9.81;           % g accel [m/s^2]
mp = 0.230;         % mass of pendulum [kg]
l = 0.6413;         % length of pendulum [m]
r = l/2;            % radius to COM of pendulum [m]
J = (1/3)*mp*l^2;   % inertia of pendulum rotating about 1 end [kg-m^2]
Y = 0.0024;         % pendulum damping [N-m*s]
mc = 0.38;          % mass of cart [kg]
c = 0.90;           % cart damping [N-s/m]

% Additional helpful parameters
Mt = mp + mc; 
Jt = J + mp*l^2;
mu = Mt * Jt - mp^2 * l^2; 

% Create the system model
A = [0 0 1 0; 0 0 0 1; 0 (mp^2 * l^2 *g)/mu (-c*Jt)/mu (-Y*Jt*l*mp)/mu; 0 (Mt*mp*g*l)/mu (-c*l*mp)/mu (-Y*Mt)/mu]; 
B = [0; 0; Jt/mu; (l*mp)/mu]; 
C = [1 0 0 0; 0 1 0 0]; 
D = [0; 0]; 

%% Compute the transfer functions associated with the system
[b,a] = ss2tf(A, B, C, D, 1);

P1 = tf(b(1,:),a) 
P2 = tf(b(2,:),a)

%% Use the control system toolbox to design a controller
controlSystemDesigner(P1)

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


