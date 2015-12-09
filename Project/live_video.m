%% Setup the Kinect acquisition variables
% Number of seconds 
N = 10; 

% Acquire data into memory before logging it
colorVid = videoinput('kinect',1); 
depthVid = videoinput('kinect',2);

% Set Kinect Properties
set([colorVid depthVid], 'FramesPerTrigger', 1);
set([colorVid depthVid], 'TriggerRepeat', Inf);
triggerconfig([colorVid depthVid], 'manual')


%% Start the color and depth device. This begins acquisition, but does not
% start logging of acquired data.
start([colorVid depthVid]);

% Trigger the devices to start logging of data.
trigger([colorVid depthVid]);

rgb_frame = getdata(colorVid); depth_frame = getdata(depthVid);

figure;
h = imshow(rgb_frame);

%%
disp('Starting data acquisition...')
t0 = tic; t = 0;
while(t < N)
    t = toc(t0); 
    trigger([colorVid depthVid]);
    rgb_frame = getdata(colorVid); 
    depth_frame = getdata(depthVid);
    set(h,'Cdata',rgb_frame);
    drawnow
end

%%
delete(colorVid)
delete(depthVid)
clear colorVid
clear depthVid
