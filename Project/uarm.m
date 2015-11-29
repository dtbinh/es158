classdef uarm
   properties
      % Basic properties of the connection with the Accelerometer
      BaudRate
      ComPort
      SerialPort
      FormatString 
   end
   methods
       function U = uarm(com_port)
         U.BaudRate = 9600;
         U.ComPort = com_port; 
         U.SerialPort = serial(U.ComPort, 'BaudRate', U.BaudRate);
         U.FormatString=['a/g/m:\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\n'];
         fopen(U.SerialPort); 
      end

      function A = calibrate(A)
      end

      function s = getDataSample(A)
      end

      function close(A)
         fclose(A.SerialPort);         
      end

      function delete(A)
        delete(A.SerialPort);
        clear A.SerialPort;
      end
   end
end
