function score = computeScore(model_name)

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