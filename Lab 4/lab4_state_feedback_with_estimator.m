
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

% Compute the full state space model, with all states output
A = [zeros(2) eye(2); -inv(M)*kappa -inv(M)*beta]; % A,B,C,D in block matrix form
B = [0; 0; inv(M)*S];    % input matrix
C = [1 0 0 0; 0 0 0 1]; 
D = [0; 0]; 
OL = ss(A, B, C, D);     % open loop system

%% Place the poles of a proportional/reference controller
p = [-4.2071 -1.4327 -1.5 -1];
K = place(A,B,p);

% Set xe to a constant, solve dynamics equation so that x_dot = 0
kr = -1/(C*inv(A-B*K)*B); 

L = place(A',C',[-1 -1.1 -1.01 -1.001])'; 

%% Run the simulink file from Matlab

%% Read in the simulink output
temp = y_simout.Data;
y = reshape(temp,60,1);
t = y_simout.Time; 
info = stepinfo(y,t); 
