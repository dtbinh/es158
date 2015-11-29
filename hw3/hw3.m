figure(1)
hold on

yss = 1; 
z0_array = [0.1, 0.4, 0.7, 0.9]; 
for i = 1:length(z0_array)
    % Define constants
    z0 = z0_array(i); 
    w0 = 1; 
    r = yss; 
    
    % Define the state space model
    a = [0 1; -w0^2 -2*z0*w0]; 
    b = w0^2*r*[0; 1]; 
    c = [1 0]; 
    sys = ss(a,b,c,0);
    
    % Compute the step response
    step(sys)
end

legend('z0=0.1', 'z0=0.4', 'z0=0.7', 'z0=0.9')