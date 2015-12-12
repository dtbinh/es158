clf
close all

%% Setup the Kinect acquisition variables
% Number of seconds 
N = 60; 

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

%% Trigger the devices to start logging of data.
trigger([colorVid depthVid]);

[rframe, rts, rmetaData] = getdata(colorVid); 
[dframe, dts, dmetaData] = getdata(depthVid);

figure; h1 = imshow(rframe); 
figure; h2 = imagesc(dmetaData.SegmentationData); 

%%
disp('Starting data acquisition...')
vcolor = VideoWriter('rframes.avi','Uncompressed AVI');
vdepth = VideoWriter('dframes.avi','Grayscale AVI');
open(vcolor)
open(vdepth)
t = 0; tic; 
while(ishandle(h1) && ishandle(h2))
    t = toc(t0); 
    % Trigger the data
    trigger([colorVid depthVid]);
    % Get the frame off the camera
    [rframe, rts, rmetaData] = getdata(colorVid); 
    [dframe, dts, dmetaData] = getdata(depthVid);
    % Write data to file 
    writeVideo(vcolor,rframe)
    writeVideo(vdepth,mat2gray(dframe, [0 2^16-1]))
    % Write to video display
    set(h1,'Cdata',rframe);
    set(h2,'Cdata',dframe)
    % draw video display
    drawnow
end
close(vcolor)
close(vdepth)

%% Clean up
delete(colorVid)
delete(depthVid)
clear colorVid
clear depthVid
