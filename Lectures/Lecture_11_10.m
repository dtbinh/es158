% Equations of motion
%% Create the transfer function s
s = tf('s'); 

% Plant dynamics
k = 1; r = 1; 
P = -k/(s^2 - r^2); 

% Controller design
k1 = 1; a = 1; b = 0.1; 
C = -k1 * (s + a) / (s + b); 

% Plot the results
figure
hold on
bodeplot(P)
bodeplot(C)
bodeplot(P*C)
legend('P', 'C', 'P*C')

figure
nyquist(P * C)

%% Control response
ks = [1 2 3];
figure
hold on

for i = 1:length(ks)
    C = -ks(i) * (s + a) / (s + b); 
    S = 1 / (1 * P*C); 
    bodeplot(S)
end

legend(['k=', num2str(ks(1))], ['k=', num2str(ks(2))], ['k=', num2str(ks(3))])


