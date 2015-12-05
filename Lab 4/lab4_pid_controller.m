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
z1 = [-2 -3.5]; 
k1 = -101;
p1 = 0; 
C_angle = zpk(z1,p1,k1); 
C_angle_kp = k1 * (-z1(1) - z1(2)); 
C_angle_ki = k1 * (-z1(1)) * (-z1(2)); 
C_angle_kd = k1; 

%% Use the control system toolbox to a design a controller for the angle
x_over_u = P_pos / (1 + P_angle * C_angle); 
if( DESIGN == 2)
    controlSystemDesigner(x_over_u)
end

%% Design a simple PID controller to control position
z2 = []; 
k2 = -42;
p2 = []; 
C_pos = zpk(z1,p1,k2); 
C_pos_kp = k2; 

%% Compute the score
computeScore('pid_controller_model')

error('Stop execution before optimization')

%% Optimize the zero placement of the PID controller
temp1 = -linspace(1.51, 4.01, 20); 
temp2 = -linspace(1.51, 4.02, 20); 
z1_ops = combvec(temp1, temp2); % All the combination for options of zeros
scores = zeros(1, length(z1_ops)); 
for i = 1:length(z1_ops)
    disp([num2str(100 * i/length(z1_ops)), ' percent complete']) 
    z1 = z1_ops(:, i)'; 
    k1 = -101;
    p1 = 0; 
    C_angle = zpk(z1,p1,k1); 
    C_angle_kp = k1 * (-z1(1) - z1(2)); 
    C_angle_ki = k1 * (-z1(1)) * (-z1(2)); 
    C_angle_kd = k1; 
    scores(i) = computeScore('pid_controller_model'); 
end

%% Compute best poles and plot
best_zeros = z1_ops(:, max(scores) == scores)';

figure(1)
plot(scores)
title('Optimization of C_{angle} zero placement')
ylabel('Score')
xlabel('Trial Number')
ylim([0 155])

%% Optimize the gain placement of the C_angle controller
k1_ops = -linspace(50,150,100);
scores = zeros(1, length(k1_ops)); 
for i = 1:length(k1_ops)
    disp([num2str(100 * i/length(k1_ops)), ' percent complete']) 
    z1 = [-1.9047 -3.8879]; 
    k1 = k1_ops(i);
    p1 = 0; 
    C_angle = zpk(z1,p1,k1); 
    C_angle_kp = k1 * (-z1(1) - z1(2)); 
    C_angle_ki = k1 * (-z1(1)) * (-z1(2)); 
    C_angle_kd = k1; 
    scores(i) = computeScore('pid_controller_model'); 
end

%% Compute best gains and plot
best_gains = k1_ops(:, max(scores) == scores)';

figure(2)
plot(k1_ops,scores)
title('Optimization of C_{angle} kp placement')
ylabel('Score')
xlabel('k_p value')
ylim([0 155])

%% Optimize the gain (kd) placement of the C_pos controller
z1 = [-1.9047 -3.8879]; 
k1 = -101;
C_angle_kp = k1 * (-z1(1) - z1(2)); 
C_angle_ki = k1 * (-z1(1)) * (-z1(2)); 
C_angle_kd = k1; 
k2_ops = -linspace(1,100,100);
scores = zeros(1, length(k2_ops)); 
for i = 1:length(k2_ops)
    disp([num2str(100 * i/length(k2_ops)), ' percent complete']) 
    C_pos_kp = k2_ops(i); 
    scores(i) = computeScore('pid_controller_model'); 
end

%% Compute best gains and plot
best_gains = k2_ops(:, max(scores) == scores)';

figure(2)
plot(k2_ops,scores)
title('Optimization of C_{pos} kp placement')
ylabel('Score')
xlabel('k_p value')
ylim([0 155])

%% Consolidate the most optimial values to date and compute score
z1 = [-3.2732 -2.02]; 
z1 = [-1.9047 -3.8879]; 
k1 = -106;
C_angle_kp = k1 * (-z1(1) - z1(2)); 
C_angle_ki = k1 * (-z1(1)) * (-z1(2)); 
C_angle_kd = k1; 

C_pos_kp = -30;

model_name = 'pid_controller_model'; 

% Run the simulink file from Matlab 
sim(model_name)   

% Read in the simulink output
% Grab theata from the workspace
theta = theta_out.Data; 
max_theta = max(theta(:));

% Grab x position data from the workspace
t = x_out.Time; 
temp = x_out.Data;
y = reshape(temp,length(t),1);
info = stepinfo(y,t, 'SettlingTimeThreshold', 0.01); 

% Compute the score
score = 155 - 10 * (100*info.Overshoot) - 2 * info.SettlingTime - 500 * max_theta; 

