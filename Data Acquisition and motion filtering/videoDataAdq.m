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

end
 

for  i = 150:-1:1
       
    foreground = ballForegroundCell{i};
    
    maskedForeground = and(foreground,ballFilterMask);
    [area,centroid,~] = step(ballBlobAnalysis, maskedForeground);
    
    
        % vector backward speed aproximation
    previousDisplacement = ballPositionArray(i+1,:) - ballPositionArray(i+2,:);
    radius = 30; 
    
    [theta,rho] = cart2pol(previousDisplacement(1),previousDisplacement(2));
    theta = theta + 0.01;
    rho   = rho - 1;
    [v1,v2] = pol2cart(theta,rho);
    predictedDisplacement = [v1,v2];
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
            ballPositionArray(i,:) = ballPositionArray(i+1,:) + predictedBallPosition ;
        end
        
    else
        ballPositionArray(i,:) = ballPositionArray(i+1,:) + predictedBallPosition ;
    end
end



%  release(videoPlayer);
%  release(videoReader);
%  delete(videoPlayer); 
end
