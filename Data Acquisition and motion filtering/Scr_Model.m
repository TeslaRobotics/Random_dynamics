clear
clc

videoSource = 'E:\video_data\R1.mp4';
[ballArray,rotorArray] = videoDataAdq(videoSource);

centerPosition = position_centered(ballArray);


ballArray(:,1) = ballArray(:,1)-centerPosition(:,1);
ballArray(:,2) = ballArray(:,2)-centerPosition(:,2);

vectorSpeedArray(:,1) = diff(ballArray(:,1));
vectorSpeedArray(:,2) = diff(ballArray(:,2));

vectorAccelerationArray(:,1) = diff(vectorSpeedArray(:,1));
vectorAccelerationArray(:,2) = diff(vectorSpeedArray(:,2));

r = sqrt(ballArray(:,1).^2 + ballArray(:,2).^2);
v = sqrt(vectorSpeedArray(:,1).^2 + vectorSpeedArray(:,2).^2);
a = sqrt(vectorAccelerationArray(:,1).^2 + vectorAccelerationArray(:,2).^2);


subplot(3,1,1);
hold on
plot(r);
%axis([0 500 0 200]);
vf = r;
[vf_len,~]=size(vf);
n  = 10;

P = polyfit(1:vf_len,vf',n);

x = 1:vf_len ;
y = polyval(P,x);
plot(x,y,'Color','r');

subplot(3,1,2);
hold on
plot(v);
%axis([0 500 0 200]);


vf = v;
[vf_len,~]=size(vf);
n  = 4;

P = polyfit(1:vf_len,vf',n);

x = 1:vf_len ;
y = polyval(P,x);
plot(x,y,'Color','r');






