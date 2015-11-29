function [ dv_dt ] = pi_function( t, v )
%PI_FUNCTION Proportional-integral control law
%   Detailed explanation goes here

% Defining constants
m = 1; 
a = 0.1; 
w = 0; 

% Defining u
w0 = 1; 
v_ref = sin(w0 * t); 
kp = 1; 
ki = 1; 
u = -kp * (v - v_ref) - ki * integral( @(t) v_ref - v, 0, t, 'ArrayValued', 1); 

dv_dt = (-a/m) * v + (1/m) * u + w; 

end

