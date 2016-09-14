
clc


for iRecord = 1:10 
clearvars -except iRecord

source = strcat(num2str(iRecord),'.mp4');
    
videoSource = strcat('E:\video_data\R',source);
[ballArray,rotorArray] = videoDataAdq(videoSource);

centerPosition = position_centered(ballArray);
ballArray(:,1) = ballArray(:,1)-centerPosition(:,1);
ballArray(:,2) = ballArray(:,2)-centerPosition(:,2);
%ballArrayCell{iRecord} = ballArray;

vectorSpeedArray(:,1) = diff(ballArray(:,1));
vectorSpeedArray(:,2) = diff(ballArray(:,2));

r = sqrt(ballArray(:,1).^2 + ballArray(:,2).^2);
v = sqrt(vectorSpeedArray(:,1).^2 + vectorSpeedArray(:,2).^2);

hold on
vf = v;
[vf_len,~]=size(vf);
n  = 10;

P = polyfit(1:vf_len,vf',n);

x = 1:vf_len ;
y = polyval(P,x);
vRef = 25;
[~,index] = min(abs(y-vRef));
x = x - index;

plot(x,y,'Color','r');

end









