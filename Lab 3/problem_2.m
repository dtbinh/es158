% wgc = 1.0335; 
% ym = pi - angle(wgc * 1j) - angle(wgc * 1j + 0.2) - angle(wgc * 1j + 0.05) + angle(wgc * 0.05 * 1j + 1)

clf

% Case 1 
kp = [0.5 0.05 0.05 0.005]; 
ki = [0.1 1 0.001 0.001];

gm = zeros(1, 4); 
pm = zeros(1, 4); 
wgm = zeros(1, 4); 
wpm = zeros(1, 4); 


figure(1)
hd(1) = tf([kp(4) ki(4)],[1 .25 .01 0]);
[gm(1),pm(1),wgm(1),wpm(1)]=margin(hd(1)); 
pzplot(feedback(hd(1), tf(1)))

%% Problem 1a
% Specify the parameters of the system
m = 1000; 
c = 50; 
b = 25; 
a = 0.2; 
T = 200; 

% Changing parameter
kp = 0.1; 

% Define the integrator function 
integrator = tf(1, [1 0]); 

% Tranfer function for the engine
A1 = tf(a); 
B1 = tf(a * T * kp); 
C1 = tf(1); 
D1 = tf(0);

temp1 = feedback(integrator, A1); 
Engine = B1 * temp1 * C1

% Transfer function for the plant
A2 = tf(c/m); 
B2 = tf(b/m); 
C2 = tf(1); 
D2 = tf(0/m);

temp2 = feedback(integrator, A2); 
Plant = B2 * temp2 * C2

% Transfer function for the entire system
System = feedback(Engine * Plant, tf(1))

% Plot the step function 
figure(1)
step(System)

% Plot the bode plot of the system
figure(2)
bode(System)

%% Problem 1c
% Specify the parameters of the system
m = 1000; 
c = 50; 
b = 25; 
a = 0.2; 
T = 200; 

% Changing parameter
ki = 0.1; 
kp = 0.5; 

% Define the integrator function 
integrator = tf(1, [1 0]); 

% Tranfer function for the engine
A1 = tf(a); 
B1 = (tf(kp) + integrator * tf(ki)) * tf(a * T);  
C1 = tf(1); 
D1 = tf(0);

temp1 = feedback(integrator, A1); 
Engine = B1 * temp1 * C1

% Transfer function for the plant
A2 = tf(c/m); 
B2 = tf(b/m); 
C2 = tf(1); 
D2 = tf(0/m);

temp2 = feedback(integrator, A2); 
Plant = B2 * temp2 * C2

% Transfer function for the entire system
System = feedback(Engine * Plant, tf(1))

% Plot the step function 
figure(1)
step(System)

% Plot the bode plot of the system
figure(2)
bode(System)



