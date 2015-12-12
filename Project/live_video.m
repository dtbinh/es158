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

figure(1); h1 = imshow(rframe); 
figure(2); h2 = imagesc(dframe); 


%% Create a bounding box around the roi for the hand and yoyo
figure(1)
title('Please define a rectangle for the yoyo bounding box:')
[x1,y1] = ginput(4); 
zbbox = struct('x',int32(min(x1)),'y',int32(min(y1)),'w',int32((max(x1)-min(x1))),'h',int32(max(y1)-min(y1)));
rectangle('Position', [zbbox.x,zbbox.y,zbbox.w,zbbox.h],...
  'EdgeColor','r','LineWidth',2 )

%%
disp('Starting data acquisition...')
vcolor = VideoWriter('rframes.avi','Uncompressed AVI');
vdepth = VideoWriter('dframes.avi','Grayscale AVI');
open(vcolor)
open(vdepth)
t = 0; t0 = tic; 
while(ishandle(h1) && ishandle(h2))
    t = toc(t0); 
    % Trigger the data
    trigger([colorVid depthVid]);
    % Get the frame off the camera
    [rframe, rts, rmetaData] = getdata(colorVid); 
    [dframe, dts, dmetaData] = getdata(depthVid);
    % Write data to file 
    writeVideo(vcolor,rframe(zbbox.y:(zbbox.y+zbbox.h),zbbox.x:(zbbox.x+zbbox.w),:))
    writeVideo(vdepth,mat2gray(dframe, [0 2^16-1]))
    % Write to video display
    set(h1,'Cdata',rframe(zbbox.y:(zbbox.y+zbbox.h),zbbox.x:(zbbox.x+zbbox.w),:));
    set(h2,'Cdata',dframe)
    % draw video display
    drawnow
end

%% 
close(vcolor)
close(vdepth)

% Clean up
delete(colorVid)
delete(depthVid)
clear colorVid
clear depthVid
