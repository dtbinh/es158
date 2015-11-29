%% Create the transfer function s
s = tf('s'); 

%% 11/3
ki = 0.1; 
m = 1; c = 0.2; k = 1; P = 1/(m*s^2 + c * s + k);
C = kp + ki / s; 
figure(1); bodeplot(P);
figure(2); bodeplot(C);
figure(3); bodeplot(P * C / (1 + P*C));
figure(4); step(P*C / (1 + P*C));

%% 11/5
kd = 10;
C = 1 + kd * s; 

m = 1; 
c = 0.2;
k = 1; 
P = 1/(m * s^2 + c * s + k); 

figure
hold on
bodeplot(P)
bodeplot(C)
bodeplot(P*C)
legend('P', 'C', 'P*C')

figure
step(P*C)

%% Dealing with lead compensation
TD = 1;
alpha = 0.1; 
C = (TD * s + 1) / (alpha * TD * s + 1); 

m = 1; 
c = 0.2;
k = 1; 
P = 1/(m * s^2 + c * s + k); 

figure
hold on
bodeplot(P)
bodeplot(C)
bodeplot(P*C)
legend('P', 'C', 'P*C')

figure
step(P*C)

%% The PI controller
kp = 0.1; 
ki = 1;
C = kp + ki / s;

m = 1; 
c = 0.2;
k = 1; 
P = 1/(m * s^2 + c * s + k); 

figure
hold on
bodeplot(P)
bodeplot(C)
bodeplot(P*C)
legend('P', 'C', 'P*C')

figure
step(P*C)

%% Lag Compensation 
alpha = 10;
Ti = 10; 
C = alpha * (Ti * s + 1) / (alpha * Ti * s + 1); 

c = 0.2;
k = 1; 
P = 1/(m * s^2 + c * s + k); 

figure
hold on
bodeplot(P)
bodeplot(C)
bodeplot(P*C)
legend('P', 'C', 'P*C')

figure
step(feedback(P*C, 1))

%% PID Controller Compensation
kp = 200; 
kd = 10; 
ki = 1; 

C_pid = kp + kd * s + ki / s; 

TD = 1;
alpha = 0.1; 
C_lead_comp = (TD * s + 1) / (alpha * TD * s + 1); 

C = C_pid * C_lead_comp;

% K=100; 
% Td = 1; 
% Ti = 10; 
% C = K/s * (Td * s + 1) * (s + Ti); 

c = 0.2;
k = 1; 
P = 1/(m * s^2 + c * s + k); 

figure
hold on
bodeplot(P)
bodeplot(C)
bodeplot(P*C)
legend('P', 'C', 'P*C')

figure
step(feedback(P*C, 1))
