function [ballPositionArray,rotorPositionArray] = videoDataAdq(url)

%url = 'E:\video_data\R2.mp4';
videoReader = vision.VideoFileReader(url);

%videoPlayer = vision.VideoPlayer;

ballBlobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
    'AreaOutputPort', true, 'CentroidOutputPort', true, ...
    'MinimumBlobArea', 4);

rotorBlobAnalysis = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
    'AreaOutputPort', true, 'CentroidOutputPort', true, ...
    'MinimumBlobArea', 40);

foregroundDetector = vision.ForegroundDetector('NumGaussians', 3,'NumTrainingFrames', 50);

for i = 1:30
    videoFrame = step(videoReader);
    step(foregroundDetector,videoFrame);
end
reset(videoReader); 

path = 'C:\Documents and Settings\Pablito\My Documents\MATLAB\Roulette\Data Acquisition and motion filtering\rsc\images\';
ballFilterMask  = imread(strcat(path, 'ball_mask.bmp'));
rotorFilterMask = imread(strcat(path, 'rotor_mask.bmp'));
ballTresholdMask = imread(strcat(path, 'tresh_mask.bmp'));


ballPositionArray = zeros(1,2);
rotorPositionArray = zeros(1,2);
i = 1;
while  ~isDone(videoReader)
    
    videoFrame = step(videoReader);    
    foreground = step(foregroundDetector,videoFrame);
    
    maskedForeground = and(foreground,ballFilterMask);
    [area,cent,~] = step(ballBlobAnalysis, maskedForeground);
    
     if ~isempty(area)
         [~,I] = max(area);
         ballPositionArray(i,:) = cent(I,:);         
     else
         ballPositionArray(i,:) = [1,1];
     end

     
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    r = videoFrame(:,:,1);
    g = videoFrame(:,:,2);
    b = videoFrame(:,:,3);
    f = g - r/2 - b/2 ;
    bw = f>0.03;
    bw = and(bw,rotorFilterMask);
    
    [area2,cent2,~] = step(rotorBlobAnalysis, bw);
    
    if ~isempty(area2)
        [~,I2] = max(area2);
        rotorPositionArray(i,:) = cent2(I2,:);
    else
        rotorPositionArray(i,:) = [1,1];
    end
    
    ballPosition = round(ballPositionArray(i,:));
    if and(ballTresholdMask(ballPosition(2),ballPosition(1)),i > 100)
        break;
    end
    
    i = i+1;
end
%  release(videoPlayer);
%  release(videoReader);
%  delete(videoPlayer); 
end
