function [ballPositionArray,rotorPositionArray] = videoDataAdq(url)

%url = 'E:\video_data\R2.mp4';
videoReader = vision.VideoFileReader(url);

% videoPlayer = vision.VideoPlayer;

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
tresholdMask    = imread(strcat(path, 'tresh_mask.bmp'));


for i = 1:20
    videoFrame = step(videoReader);
    step(foregroundDetector,videoFrame); 
end

reset(videoReader);
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
        [~,I] = max(area);
        rotorPositionArray(i,:) = centroid(I,:);
    else
        rotorPositionArray(i,:) = [1,1];
    end
    
    i = i + 1;
end


ballPositionArray = ones(1,2);

for  i = 150:numel(ballForegroundCell)
       
    foreground = ballForegroundCell{i};
    
    maskedForeground = and(foreground,ballFilterMask);
    [area,centroid,~] = step(ballBlobAnalysis, maskedForeground);
    
     if ~isempty(area)
         [~,I] = max(area);
         ballPositionArray(i,:) = centroid(I,:);         
     else
         ballPositionArray(i,:) = [1,1];
     end
    ballPosition = round(ballPositionArray(i,:)); 
    if tresholdMask(ballPosition(2),ballPosition(1))
        break;
    end
end
 

for  i = 149:-1:1
       
    foreground = ballForegroundCell{i};
    
    maskedForeground = and(foreground,ballFilterMask);
    [area,centroid,~] = step(ballBlobAnalysis, maskedForeground);
    
    
        % vector backward speed aproximation
    previousDisplacement = ballPositionArray(i+1,:) - ballPositionArray(i+2,:);       
    radius = 30;
    rotation = 20;
    R = [cosd(rotation),-sind(rotation);sind(rotation),cosd(rotation)];    
    predictedDisplacement = previousDisplacement*R;
    predictedBallPosition = ballPositionArray(i+1,:) + predictedDisplacement ;    
       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ~isempty(area)
        [~,sortIndexArray] = sort(area, 'descend');        
         sortCentroidArray = centroid(sortIndexArray(:),:);
        
        for j = 1:numel(sortIndexArray);
            b = 0;
            if (norm(sortCentroidArray(j,:)-predictedBallPosition) < radius)
                break;
            end
            b = 1;
        end
        
        if ~b
            I = sortIndexArray(j);
            ballPositionArray(i,:) = centroid(I,:);            
        else
            ballPositionArray(i,:) = predictedBallPosition ;
        end
        
    else
        ballPositionArray(i,:) = predictedBallPosition ;
    end
    
%     circle_vector = ballPositionArray(i,:);
%     circle_vector(3)= radius;
%     result = insertShape(maskedForeground*0.5, 'circle', circle_vector, 'Color', 'green','LineWidth', 1); 
%     step(videoPlayer, result); 
%     pause(0.1);
end


%  release(videoPlayer);
%  release(videoReader);
%  delete(videoPlayer); 
end
