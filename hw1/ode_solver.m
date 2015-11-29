% How to Solve ordinary differential equations in MATLAB.

options=odeset('RelTol',1e-10,'AbsTol',10^-10); %Determine the error of each step in Runge-Kuta method
R = 1; % ohm
L = 0.1; %H
C = 0.2; %F
Vs = 1; %V
Fun = @(t,Y) [0 1; -1/(L*C) -1/(R*C)] * [Y(1); Y(2)] + [0; 1/(L*C)] * Vs;  %Define the state space model, where dy=F(t,y).
[x,y]=ode45(Fun,[0 3],[0;0]); %ODE solver
subplot(2,1,1)  
title('Solution of a second order ODE:y(1)')
plot(x,y(:,1),'r')
hold on
subplot(2,1,2)
plot(x,y(:,2))
title('Solution of a second order ODE:y(2)')
%===============================================



