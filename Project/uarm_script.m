%% Initialize the ComPort
com_port = 'COM5';
BaudRate = 9600;
SerialPort = serial(com_port, 'BaudRate', BaudRate);
fopen(SerialPort);

%% Issue a command to the uarm
pause(2);
fprintf(SerialPort, '2');
pause(2); 

%% Tell the uarm to move to a coordinate in space
fprintf(SerialPort, '1');
coord = [0 -13 10]; 
fprintf(SerialPort,['%i %i %i'],coord,'sync');

%% Close the comport
fclose(SerialPort);
delete(SerialPort);
clear SerialPort
