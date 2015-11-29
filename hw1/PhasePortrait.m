%PhasePortrait for d^2Vc/dt^2 = -1/RC * dVc/dt - 1/LC * Vc + 1/LC * Vs

% f = @(t,Y) [Y(2); -sin(Y(1))];  %Define the state space model, where dy=F(t,y).
R = 1; % ohm
L = 0.1; %H
C = 0.2; %F
Vs = 1; %V
f = @(t,Y) [0 1; -1/(L*C) -1/(R*C)] * [Y(1); Y(2)] + [0; 1/(L*C)] * Vs;  %Define the state space model, where dy=F(t,y).
y1 = linspace(-2,8,20);    %Creates 20 equi-distance points from -2 to 8.
y2 = linspace(-2,2,20);    %Creates 20 equi-distance points from -2 to 2.
[x,y] = meshgrid(y1,y2);  %Creates a grid from points y1 and y2. 

u = zeros(size(x));  %Create a vector to save the velocity values at each point of the grid
v = zeros(size(x));  %Create a vector to save the velocity values at each point of the grid

t=0;
for i = 1:numel(x)
    Yprime = f(t,[x(i); y(i)]);    %Computing the values of the velocity at each point of the grid (line 6).
    u(i) = Yprime(1);              %Saving the result in vector u
    v(i) = Yprime(2);              %Saving the result in vector v
end

quiver(x,y,u,v,'r');     %Plotting the phase portrait with a red color 'r'.
xlabel('y_1')
ylabel('y_2')
