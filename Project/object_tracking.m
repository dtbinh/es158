clf
close all

%% Setup the Kinect acquisition variables
% Number of seconds 
N = 1000; 

% Acquire data into memory before logging it
colorVid = videoinput('kinect',1); 
depthVid = videoinput('kinect',2);

% Set Kinect Properties
set([colorVid depthVid], 'FramesPerTrigger', 1);
set([colorVid depthVid], 'TriggerRepeat', Inf);
triggerconfig([colorVid depthVid], 'manual')

% Start the color and depth device. This begins acquisition, but does not
% start logging of acquired data.
start([colorVid depthVid]);

%% Trigger the devices to start logging of data.
trigger([colorVid depthVid]);

[rframe, rts, rmetaData] = getdata(colorVid); 
[dframe, dts, dmetaData] = getdata(depthVid);

figure; h1 = imshow(rframe);  

%% Create a bounding box around the roi for the hand and yoyo
title('Please define a rectangle for the yoyo bounding box:')
[x1,y1] = ginput(4); 
zbbox = struct('x',min(x1),'y',min(y1),'w',(max(x1)-min(x1)),'h',max(y1)-min(y1));
rectangle('Position', [zbbox.x,zbbox.y,zbbox.w,zbbox.h],...
  'EdgeColor','r','LineWidth',2 )

title('Please define a rectangle for the hand bounding box:')
[x2,y2] = ginput(4); 
hbbox = struct('x',min(x2),'y',min(y2),'w',(max(x2)-min(x2)),'h',max(y2)-min(y2));
rectangle('Position', [hbbox.x,hbbox.y,hbbox.w,hbbox.h],...
  'EdgeColor','b','LineWidth',2 )

%% Find the pixels that are at the depth range of the yoyo and arm
zminDepth = 1950; % mm
zmaxDepth = 2300; % mm
zcolorMask = poly2mask(x1, y1, size(rframe,1), size(rframe,2));
zdepthMask = (dframe > zminDepth & dframe < zmaxDepth) & zcolorMask;
figure; h2 = imshow(zdepthMask);

hminDepth = 1950; % mm
hmaxDepth = 2300; % mm
hcolorMask = poly2mask(x2, y2, size(rframe,1), size(rframe,2));
hdepthMask = (dframe > hminDepth & dframe < hmaxDepth) & hcolorMask;
figure; h3 = imshow(hdepthMask);

%% Perform a morphological close operation on the image on a GPU.
% se = strel('disk',5);
% se2 = strel('disk',5);
% 
% % Find the z centroid
% zcloseBW = imopen(zdepthMask,se);
% figure; h3 = imshow(zcloseBW);
% zCC = bwconncomp(zcloseBW);
% zS = regionprops(zCC,'Centroid');
% 
% % Find the h centroid
% hcloseBW = imopen(hdepthMask,se);
% figure; h4 = imshow(hcloseBW);
% hCC = bwconncomp(hcloseBW);
% hS = regionprops(hCC,'Centroid');
% 
%% Try edge detection
colorMask = imopen((zcolorMask | hcolorMask), se); 
depthMask = imopen((zdepthMask | hdepthMask), se);
BW1 = edge(dframe,'sobel');
BW2 = edge(dframe,'canny'); 
figure;
imshowpair(BW1,BW2,'montage')
title('Sobel Filter Canny Filter');

%% Real time object tracking
% Allocate memory for keeping track of object position
handPos = zeros(1,N);
yoyoPos = zeros(1,N);
figure; 
subplot(1,2,1); lh1 = line(nan,nan); title('Z position')
subplot(1,2,2); lh2 = line(nan,nan); title('H position')

%% Try a slightly different approach
% obj = setupSystemObjects();
% tracks = initializeTracks(); % Create an empty array of tracks.
% nextId = 1; % ID of the next track
% [centroids, bboxes, mask] = detectObjects(obj,dframe); 

% % Select the colored region to track
% disp('Setting up the object tracker...')
% addpath('..\..\es100\Kinect')
% tracker = initializeTracker(rframe);

%% Start loop
i = 1; t0 = tic; t = 0; % Initialize loop measures
while (ishandle(h1) && ishandle(h2))
    trigger([colorVid depthVid]);
    rframe = getdata(colorVid); dframe = getdata(depthVid);
    set(h1,'Cdata',rframe);
    
    % Find yoyo
    zdepthMask = (dframe > zminDepth & dframe < zmaxDepth) & zcolorMask;
    zcloseBW = imopen(zdepthMask,se);
    zCC = bwconncomp(zcloseBW);
    zS = regionprops(zCC,'Centroid');
    if(~isempty(zS))
        yoyoPos(i) = zS(1).Centroid(1)';
    elseif(i > 1)
        yoyoPos(i) = yoyoPos(i-1); 
    else
        yoyoPos(i) = 0; 
    end
    set(lh1, 'XData', 1:i,'YData', yoyoPos(1:i)); 
    set(h2,'Cdata',zcloseBW);
    
    % Find the hand
    hdepthMask = (dframe > hminDepth & dframe < hmaxDepth) & hcolorMask;
    hcloseBW = imopen(hdepthMask,se);
    hCC = bwconncomp(hcloseBW);
    hS = regionprops(hCC,'Centroid');
    if(~isempty(hS))
        handPos(i) = hS(1).Centroid(1)';
    elseif(i > 1)
        handPos(i) = handPos(i-1); 
    else
        handPos(i) = 0; 
    end
    set(lh2, 'XData', 1:i,'YData', handPos(1:i));
    set(h3,'Cdata',zcloseBW);
    
    % Update state variables and draw graphs
    drawnow
    i = mod(i,N)+1; 
end

%% Clean up
delete(colorVid)
delete(depthVid)
clear colorVid
clear depthVid


