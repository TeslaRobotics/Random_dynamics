%% Detecting and counting moving objects in video
% Copyright 2014 The MathWorks, Inc.

%% Clear Workspace
clear all; %#ok<CLSCR>
close all
clc;
%% Read Video
record = 'R2.mp4';
url = strcat('E:\video_data\',record);
videoReader = vision.VideoFileReader(url);  

%% Create Video Player
videoPlayer = vision.VideoPlayer;

%% Create blob analysis object 
%Blob analysis object further filters the detected foreground by rejecting blobs which contain fewer
% than 150 pixels.
blobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
    'AreaOutputPort', true, 'CentroidOutputPort', true, ...
    'MinimumBlobArea', 4,'MaximumBlobArea',600);

%% Create Foreground Detector  (Background Subtraction)
foregroundDetector = vision.ForegroundDetector('NumGaussians', 3,'NumTrainingFrames', 50);


%% Run on first 75 frames to learn background
for i = 1:30
    videoFrame = step(videoReader);    
    step(foregroundDetector,videoFrame);    
end
reset(videoReader); 
%% ROI ellipse
h =imshow(videoFrame);

e2 = imellipse(gca,[153 7 339 346]);
BW2 = createMask(e2,h);

e3 = imellipse(gca,[183.34 34.66 277.23 285.43]);
BW3 = createMask(e3,h);

ex = imellipse(gca,[167.34 20.66 309.23 316.43]);
BWx = createMask(ex,h);

BW1 = and(BW2,not(BW3));
%% Backwards calculation recording step

for i = 1:150
    videoFrame = step(videoReader);    
    S{i} = step(foregroundDetector,videoFrame) & BW1;    
end

%% Loop through video
P = zeros(1,2);
i = 1;


while  ~isDone(videoReader)
    %Get the next frame
    videoFrame = step(videoReader);    
    
    %Detect foreground pixels
    foreground = step(foregroundDetector,videoFrame);
    % Perform morphological filtering
    %cleanForeground = imopen(foreground, strel('Disk',1));
    cleanForeground = foreground ;
    cleanForeground = and(cleanForeground,BW1);
    %cleanForeground = imopen(cleanForeground, strel('Disk',1));
            
    % Detect the connected components with the specified minimum area, and
    % compute their bounding boxes
    [area,cent,bbox] = step(blobAnalysis, cleanForeground);
    
    % aceptar solo las areas con una distancia menor a minimum distance
    % emepzando desde la mayor
    
    if ~isempty(area)
        
        [~,I] = max(area);        
        P(i,:) = cent(I,:);
    else
        P(i,:) = [1,1];
    end
    
    centroid_maxArea = P(i,:);
    circle_vector = centroid_maxArea;
    circle_vector(3)= 5;
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    % end when ball start falling
    Pi = round(P(i,:)); %index position
    if and(BWx(Pi(2),Pi(1)),i > 100)
        break;
    end
    % Draw bounding boxes around the detected cars
    result = insertShape(videoFrame, 'circle', circle_vector, 'Color', 'green','LineWidth', 4);


    % Display output 
    step(videoPlayer, result); 
    i = i+1;
    %pause(1)
end

%% Backwards calculation step

P_back = ones(150,2);
P_back(151,:) = P(1,:);
P_back(152,:) = P(2,:);


for j = 150:-1:1
    recorded_clean_frame = S{j};
    [area2,cent2,bbox2] = step(blobAnalysis, recorded_clean_frame);
    
    % vector backward speed aproximation
    v = P_back(j+1,:) - P_back(j+2,:);
    d = 30;    
    [theta,rho] = cart2pol(v(1),v(2));
    theta = theta + 0.01;
    rho   = rho - 1;
    [v1,v2] = pol2cart(theta,rho);
    v = [v1,v2];
    % predicted backward position
    P_back_pred = P_back(j+1,:) + v ;    
       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ~isempty(area2)
        [~,sort_index] = sort(area2, 'descend');        
         sort_centroids = cent2(sort_index(:),:);
        
        for k = 1:numel(sort_index);
            b = 0;
            if (norm(sort_centroids(k,:)-P_back_pred) < d)
                break;
            end
            b = 1;
        end
        
        if ~b
            I2 = sort_index(k);
            P_back(j,:) = cent2(I2,:);            
        else
            P_back(j,:) = P_back(j+1,:) + v ;
        end
        
    else
        P_back(j,:) = P_back(j+1,:) + v ;
    end

end

%% visualization
reset(videoReader);
for j = 1:150
       % visualization
    
    videoFrame = step(videoReader); 
    
        % vector backward speed aproximation
    v = P_back(j+1,:) - P_back(j+2,:);
    d = 30;    
    [theta,rho] = cart2pol(v(1),v(2));
    theta = theta - 0.1;
    rho   = rho - 1;
    [v1,v2] = pol2cart(theta,rho);
    v = [v1,v2];    
    
    % predicted backward position
    P_back_pred = P_back(j+1,:) + v ;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    circle_vector = P_back_pred;
    circle_vector(3)= d;    
    result = insertShape(videoFrame, 'circle', circle_vector, 'Color', 'green','LineWidth', 1);
    
    circle_vector = P_back(j,:);
    circle_vector(3)= 5;     
    result = insertShape(result, 'circle', circle_vector, 'Color', 'red','LineWidth', 4);
    
    line_vector = horzcat(P_back(j+1,:),P_back(j+1,:)+v);
    result = insertShape(result, 'Line', line_vector, 'Color', 'blue','LineWidth', 1);
    
    step(videoPlayer, result); 
    pause(0.05) 
    
end


%% union of the two steps
P = vertcat(P_back(1:150,:),P);
%transform pixels to postition

% P(:,1) = P(:,1) - 1;
% P(:,2) = 288 - P(:,2);
% plot(P(450:500,1),P(450:500,2));
plot(P);

%% release video reader and writer
release(videoPlayer);
release(videoReader);

delete(videoPlayer); % delete will cause the viewer to close









