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

path = 'C:\Documents and Settings\Pablito\My Documents\MATLAB\Roulette\Data Acquisition and motion filtering\rsc\images\';
ballFilterMask  = imread(strcat(path, 'ball_mask.bmp'));
rotorFilterMask = imread(strcat(path, 'rotor_mask.bmp'));
ballTresholdMask = imread(strcat(path, 'tresh_mask.bmp'));


i = 1;
rotorPositionArray = ones(1,2);
while ~isDone(videoReader)
    videoFrame = step(videoReader);
    ballForegroundCell{i} = step(foregroundDetector,videoFrame);   
    
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    r = videoFrame(:,:,1);
    g = videoFrame(:,:,2);
    b = videoFrame(:,:,3);
    f = g - r/2 - b/2 ;
    bw = f>0.03;
    bw = and(bw,rotorFilterMask);
    
    [area,centroid,~] = step(rotorBlobAnalysis, bw);
    
    if ~isempty(area)
        [~,I2] = max(area);
        rotorPositionArray(i,:) = centroid(I2,:);
    else
        rotorPositionArray(i,:) = [1,1];
    end
    
    i = i + 1;
end


ballPositionArray = zeros(1,2);

for  j = 1:numel(ballForegroundCell)
       
    foreground = ballForegroundCell{j};
    
    maskedForeground = and(foreground,ballFilterMask);
    [area,centroid,~] = step(ballBlobAnalysis, maskedForeground);
    
     if ~isempty(area)
         [~,I] = max(area);
         ballPositionArray(i,:) = centroid(I,:);         
     else
         ballPositionArray(i,:) = [1,1];
     end
     
    ballPosition = round(ballPositionArray(i,:));
    if and(ballTresholdMask(ballPosition(2),ballPosition(1)),j > 100)
        break;
    end
 end
%  release(videoPlayer);
%  release(videoReader);
%  delete(videoPlayer); 
end
