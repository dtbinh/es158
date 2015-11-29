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

%% Stability Analysis

figure, hold on
pzmap(OL)

figure, nyquist(OL); 
% For x output: no net encirclements, but 1 pole in RHP means unstable
% For th output: no net encirclements, but 1 pole in RHP means unstable

Cx = [1 0 0 0]; Cth = [0 1 0 0];

[Numx, Denx] = ss2tf(A, B, Cx, 0);
[Numth, Denth] = ss2tf(A, B, Cth, 0);

TFx = tf(Numx, Denx);
TFth = tf(Numth, Denth);

figure, subplot(2,1,1), bode(TFx), grid on
subplot(2,1,2), bode(TFth), grid on

%% Controllability and Observability

Ctr = [B, A*B, A^2*B, A^3*B];
rank(Ctr); % full rank

Obsx = [Cx; Cx*A; Cx*A^2; Cx*A^3];
rank(Obsx); % full rank

Obsth = [Cth; Cth*A; Cth*A^2; Cth*A^3];
rank(Obsth); % NOT FULL RANK


