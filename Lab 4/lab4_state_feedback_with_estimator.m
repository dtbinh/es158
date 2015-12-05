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
p = [-1.5 -1.4 -0.51 -0.5]; % new: [-4 -6.02 -6.03 -6.04]; 
K = place(A,B,p);

% Set xe to a constant, solve dynamics equation so that x_dot = 0
kr = -1/(C*inv(A-B*K)*B); 

L = place(A',C',[-1 -1.1 -1.01 -1.001])'; 

%% Run the simulink file from Matlab
model = 'state_feedback_with_estimator'; 
sim(model)   

%% Read in the simulink output
% Grab theata from the workspace
theta = theta_out.Data; 
max_theta = max(theta(:));

% Grab x position data from the workspace
t = x_out.Time; 
temp = x_out.Data;
y = reshape(temp,length(t),1);
info = stepinfo(y,t, 'SettlingTimeThreshold', 0.01); 

% Compute the score
score = 155 - 10 * (100*info.Overshoot) - 2 * info.SettlingTime - 500 * max_theta 

error('Stopping execution before optimization')

%% Run an optimization on the placement of the poles of the system
p1 = linspace(.501,1.51, 5); 
p2 = linspace(.502,1.52, 5);  
p3 = linspace(.503,1.53, 5); 
p4 = linspace(.504,1.54, 5);  
pc = combvec(p1, p2, p3, p4); 
pc = -pc; 

scores = zeros(1, length(pc)); 
for i = 1:length(pc)
    disp([num2str(100 * i/length(pc)), ' percent complete']) 
    p = pc(:, i)'; 
    K = place(A,B,p);
    scores(i) = computeScore('state_feedback_with_estimator'); 
end

% Compute and plot
best_poles = pc(:, max(scores) == scores)';

figure(1)
plot(scores)
title('Optimization of pole placement')
ylabel('Score')
xlabel('Trial Number')
