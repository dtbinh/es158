%% Create the plant model
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

%% Root locus design
rlocus(P_pend)
title('Root Locus of Plant (under Proportional Control)')

%% Place a pole at the origin to cancel the plant zero at the origin 
C = 1/s;
rlocus(C*P_pend)
title('Root Locus with Integral Control')

%% Find the zeros and poles to start thinking about how to draw the root
% locus branches into the left-half plane
zeros = zero(C*P_pend)
poles = pole(C*P_pend)

z = sym('z')
% Compute the intersect point of another zero
num = (sum(poles) - (sum(zeros)-z))
den = (length(poles) - (length(zeros)+1))

%% PID Control
z = [-3 -4];
p = 0;
k = 1;
C = zpk(z,p,k);
rlocus(C*P_pend)
title('Root Locus with PID Controller')

[k,poles] = rlocfind(C*P_pend)

K = 20;
T = feedback(P_pend,K*C);
impulse(T)
title('Response of Pendulum Angle to an Impulse Disturbance under PID Control');