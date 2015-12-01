%% Lab 3 Solutions

clear all, close all

% State vector: [x theta x_dot theta_dot]^T

% Define theta as angle from vertical (standing pendulum)
% theta = 0 means pendulum STANDING inverted
% x is position of cart along x axis
DESIGN = 0; 

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

% Compute the full state space model, with all states output
A = [zeros(2) eye(2); -inv(M)*kappa -inv(M)*beta]; % A,B,C,D in block matrix form
B = [0; 0; inv(M)*S];    % input matrix
C = [1 0 0 0; 0 1 0 0];  % output matrix (select pos and angle)
D = [0; 0];              % no OL feedthrough
OL = ss(A, B, C, D);     % open loop system

%% Compute the numerator and denominator of the plants
[b, a] = ss2tf(A,B,C,D); 

b(2,:) = b(2,:) .* [1 1 1 0 0]; % Get rid of numeric errors

% Adjust the numerator and denominator accordingly
P_pos = tf(b(1,:),a);
P_angle = tf([0 b(2,1:4)],[0 a(1:4)]); % cancel the s from the numerator and denominator

%% Use the control system toolbox to design a controller for the angle
if( DESIGN == 1)
    controlSystemDesigner(P_angle)
end

%% Design a very simple PID controller
z1 = [-1/0.94 -1/0.55]; 
k1 = -65.439;
p1 = 0; 
C_angle = zpk(z1,p1,k1); 

%% Use the control system toolbox to a design a controller for the angle
x_over_u = P_pos / (1 + P_angle * C_angle); 
if( DESIGN == 2)
    controlSystemDesigner(x_over_u)
end

%% Design a simple PID controller to control position
z1 = -1; 
k1 = -10;
p1 = []; 
C_pos = zpk(z1,p1,k1); 
