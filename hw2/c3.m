a = [0 1; -100 -30]; 
b = [0; 100]; 
c = [1 0]; 

sys = ss(a, b, c, 0); 
impulse(sys)