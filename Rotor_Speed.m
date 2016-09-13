
%% Clear Workspace
clear all; %#ok<CLSCR>
close all
clc;
%% Read Video
record = 'R1.mp4';
url = strcat('E:\video_data\',record);
videoReader = vision.VideoFileReader(url);

%% Create Video Player
videoPlayer = vision.VideoPlayer;

%% Create blob analysis object 
%Blob analysis object further filters the detected foreground by rejecting blobs which contain fewer
% than 150 pixels.
blobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', false, ...
    'AreaOutputPort', false, 'CentroidOutputPort', true, ...
    'MinimumBlobArea', 40);

%% ROI ellipse

videoFrame = step(videoReader);
h =imshow(videoFrame);
e2 = imellipse(gca,[209 62 228 232]);
BW2 = createMask(e2,h);
e3 = imellipse(gca,[234 90 179 175]);
BW3 = createMask(e3,h);
BW1 = and(BW2,not(BW3));

%% Run on first 75 frames to learn background

P = zeros(1,2);
i = 1;

while ~isDone(videoReader)
    videoFrame = step(videoReader);
    
    r = videoFrame(:,:,1);
    g = videoFrame(:,:,2);
    b = videoFrame(:,:,3);
    f = g - r/2 - b/2 ;
    bw = f>0.03;
    bw = and(bw,BW1);
    
    % Detect the connected components with the specified minimum area, and
    % compute their bounding boxes
    bbox = step(blobAnalysis, bw);
    
    %fill the positions vector
    if ~isempty(bbox)
        P(i,:) = bbox;    
    else
        P(i,:) = [0,0];
    end
    i = i + 1;
    
    bbox(3)= 5;
    
    % Draw bounding boxes around the detected cars
    result = insertShape(videoFrame, 'circle', bbox, 'Color', 'red','LineWidth', 4);

    % Display output 
    step(videoPlayer, result);
    
end

plot(P);

%% release video reader and writer
release(videoPlayer);
release(videoReader);

delete(videoPlayer); % delete will cause the viewer to close

