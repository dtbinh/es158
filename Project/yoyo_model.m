%% Define the basic parameters of the system
m = 100; % mass of the yoyo (g)
r = 0.50; % radius of the yoyo (cm)
T = 1.90/2; % 2T is the period of the hand motion (s)
Tbar = 0.160; % Length of the stationary period of hand motion (s)
epsilon=195; % Friction coefficient (g * cm / s)
I=306; % Rotational inertia of the yoyo (g * cm^2)
g=(100)*9.81; % gravity (cm/s^2)

% Define the parameters of the model
h0 = 0; 
alpha = 1; 
a_k = [136.9, -366.5, 174.8, -49.06, 1.883]; 
pi_k = [1, 2, 3, 4, 5]*pi/T; 

% Define the time vector
dt = 0.01;  
N = 5; % number of seconds
t = linspace(0, N, N/dt);  

% Calculate the trajectory of the hand
nu2dot = zeros(1, length(t)); 
for i=1:5
    nu2dot = nu2dot + a_k(i)*sin(pi_k(i)*t); 
end
nu1dot = cumtrapz(t,nu2dot); 
nu = h0 + cumtrapz(t,nu1dot); 

% 
C1 = 0; 
C2 = 0; 
beta = r*epsilon/(I+m*r^2); 
gamma = m*r/(I+m*r^2); 

foft = zeros(1,length(t)); 
for i = 1:5
    foft = foft - a_k(i).*(pi_k(i)*sin(pi_k(i)*t)+beta*cos(pi_k(i)*t))/(pi_k(i)*(pi_k(i)^2+beta^2));
end

theta = C1 + C2*exp(-beta*t) + (m*g/epsilon)*t + alpha*gamma*foft; 