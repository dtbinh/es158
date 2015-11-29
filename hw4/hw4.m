clf

% Problem 2a
% 1/(s+a)(s + 10a)
figure(1)
hold on


b = 1; 
a = [10*k^2 11*k 1]; 
[h, w] = freqz(b, a, 256, 2000); 


% Problem 2b
% 1 + s/a / (s + 10a)

% Problem 2c
% s + a/(s + 2a)

% Problem 2d
% s^2 + 2dw + w^2