% v' = (-a/m) + u + w

time_period = linspace(0, 2*pi, 1000); 
initial = 0; 
[t, v] = ode45(@pi_function, time_period, initial); 

