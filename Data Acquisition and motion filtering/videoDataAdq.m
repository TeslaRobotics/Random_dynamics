function [P_ball,P_rotor] = videoDataAdq(url)
%% Clear Workspace
%clear all;
%close all
%clc;
%% Read Video
%url = 'E:\video_data\R2.mp4';
videoReader = vision.VideoFileReader(url);

%% Create Video Player
%videoPlayer = vision.VideoPlayer;

%% Create Foreground Detector  (Background Subtraction)
foregroundDetector = vision.ForegroundDetector('NumGaussians', 3,'NumTrainingFrames', 50);

%% Run on first 75 frames to learn background
for i = 1:30
    videoFrame = step(videoReader);
    step(foregroundDetector,videoFrame);
end
videoReader = vision.VideoFileReader(url);

%% Create blob analysis objects
%Blob analysis object further filters the detected foreground by rejecting blobs which contain fewer
% than 150 pixels.
blobAnalysis_ball = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
    'AreaOutputPort', true, 'CentroidOutputPort', true, ...
    'MinimumBlobArea', 4);

blobAnalysis_rotor = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
    'AreaOutputPort', true, 'CentroidOutputPort', true, ...
    'MinimumBlobArea', 40);

%% ROI ellipses
path = 'C:\Documents and Settings\Pablito\My Documents\MATLAB\Roulette\Data Acquisition and motion filtering\rsc\images\';
ball_BW1  = imread(strcat(path, 'ball_mask.bmp'));
rotor_BW1 = imread(strcat(path, 'rotor_mask.bmp'));
tresh_BW1 = imread(strcat(path, 'tresh_mask.bmp'));



%% Loop through video
P_ball = zeros(1,2);
P_rotor = zeros(1,2);
i = 1;
while  ~isDone(videoReader)
    
    %Get the next frame
    videoFrame = step(videoReader);
    
    
    % Ball Calculations
    
    %Detect foreground pixels
    foreground = step(foregroundDetector,videoFrame);
    % Perform morphological filtering
    %cleanForeground = imopen(foreground, strel('Disk',1));
    cleanForeground = foreground ;
    cleanForeground = and(cleanForeground,ball_BW1);
    %cleanForeground = imopen(cleanForeground, strel('Disk',1));
    
    % Detect the connected components with the specified minimum area, and
    % compute their bounding boxes
    [area,cent,~] = step(blobAnalysis_ball, cleanForeground);
    
    % aceptar solo las areas con una distancia menor a minimum distance
    % emepzando desde la mayor  
     if ~isempty(area)
         [~,I] = max(area);
         P_ball(i,:) = cent(I,:);         
     else
         P_ball(i,:) = [1,1];
     end

     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
     
    % Rotor calculations
    r = videoFrame(:,:,1);
    g = videoFrame(:,:,2);
    b = videoFrame(:,:,3);
    f = g - r/2 - b/2 ;
    bw = f>0.03;
    bw = and(bw,rotor_BW1);
    
    % Detect the connected components with the specified minimum area, and
    % compute their bounding boxes
    [area2,cent2,~] = step(blobAnalysis_rotor, bw);
    
   
    %Calculate the centroid of the mayor area   
    
    %fill the positions vector
    if ~isempty(area2)
        [~,I2] = max(area2);
        P_rotor(i,:) = cent2(I2,:);
    else
        P_rotor(i,:) = [1,1];
    end

    
    
    %Display output
    
    %centroid_maxArea = P_ball(i,:);
    %centroid_maxArea2 = P_rotor(i,:);
    %circle_vector = centroid_maxArea;
    %circle_vector2 = centroid_maxArea2;
    %circle_vector(3)= 5;
    %circle_vector2(3)= 5;
    %result_ball = insertShape(videoFrame, 'circle', circle_vector, 'Color', 'green','LineWidth', 4);
    %result_rotor = insertShape(result_ball, 'circle', circle_vector2, 'Color', 'red','LineWidth', 4);
    %step(videoPlayer, result_rotor);
    
    Pi = round(P_ball(i,:)); %index position
    if and(tresh_BW1(Pi(2),Pi(1)),i > 100)
        break;
    end
    
    i = i+1;
end
% subplot(2,1,1);
% plot(P_ball);
% subplot(2,1,2);
% plot(P_rotor);

%% release video reader and writer
% release(videoPlayer);
% release(videoReader);
% delete(videoPlayer); % delete will cause the viewer to close


end
