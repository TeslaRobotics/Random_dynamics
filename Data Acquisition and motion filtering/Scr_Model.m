clear
clc

videoSource = 'E:\video_data\R1.mp4';
[vectorPositionArray,rotorPositionArray] = videoDataAdq(videoSource);

centerPosition = position_centered(vectorPositionArray);


vectorPositionArray(:,1) = vectorPositionArray(:,1)-centerPosition(:,1);
vectorPositionArray(:,2) = vectorPositionArray(:,2)-centerPosition(:,2);

vectorSpeedArray(:,1) = diff(vectorPositionArray(:,1));
vectorSpeedArray(:,2) = diff(vectorPositionArray(:,2));

vectorAccelerationArray(:,1) = diff(vectorSpeedArray(:,1));
vectorAccelerationArray(:,2) = diff(vectorSpeedArray(:,2));

positionArray = sqrt(vectorPositionArray(:,1).^2 + vectorPositionArray(:,2).^2);
speedArray = sqrt(vectorSpeedArray(:,1).^2 + vectorSpeedArray(:,2).^2);
AccelerationArray = sqrt(vectorAccelerationArray(:,1).^2 + vectorAccelerationArray(:,2).^2);


subplot(3,1,1);
hold on
plot(positionArray);
axis([0 500 0 200]);
vf = positionArray;
[vf_len,~]=size(vf);
n  = 10;

P = polyfit(1:vf_len,vf',n);

x = 1:vf_len ;
y = polyval(P,x);
plot(x,y,'Color','r');

subplot(3,1,2);
hold on
plot(speedArray);
axis([0 500 0 200]);


vf = speedArray;
[vf_len,~]=size(vf);
n  = 4;

P = polyfit(1:vf_len,vf',n);

x = 1:vf_len ;
y = polyval(P,x);
plot(x,y,'Color','r');
axis([0 500 0 75]);






