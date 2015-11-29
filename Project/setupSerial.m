function [accelerometer, flag] = setupSerial(ComPort)
    flag = 1;
    accelerometer.s = serial(ComPort);
    set(accelerometer.s,'DataBits',8 );
    set(accelerometer.s,'StopBits',1 );
    set(accelerometer.s,'BaudRate', 9600);
    set(accelerometer.s,'Parity', 'none' );
    fopen(accelerometer.s);
    
    fprintf(accelerometer.s,'%c','a');
    
    a = 'b';
    while (a~='a')
        a = fread(accelerometer.s,1,'uchar');
    end
    if (a=='a')
        display('serial read');
    end 
        mbox = msgbox('Serial Communication setup'); uiwait(mbox);
        fscanf(accelerometer.s,'%u');
end