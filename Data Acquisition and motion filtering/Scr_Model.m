clear
clc

vid = 'E:\video_data\R1.mp4';
[P_ball,P_rotor] = videoDataAdq(vid);

c = position_centered(P_ball);


P_c(:,1) = P_ball(:,1)-c(1);
P_c(:,2) = P_ball(:,2)-c(2);

V_c(:,1) = diff(P_c(:,1));
V_c(:,2) = diff(P_c(:,2));

A_c(:,1) = diff(V_c(:,1));
A_c(:,2) = diff(V_c(:,2));

r = sqrt(P_c(:,1).^2 + P_c(:,2).^2);
v = sqrt(V_c(:,1).^2 + V_c(:,2).^2);
a = sqrt(A_c(:,1).^2 + A_c(:,2).^2);


subplot(3,1,1);
hold on
plot(r);
axis([0 500 0 200]);
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
axis([0 500 0 200]);


vf = v;
[vf_len,~]=size(vf);
n  = 4;

P = polyfit(1:vf_len,vf',n);

x = 1:vf_len ;
y = polyval(P,x);
plot(x,y,'Color','r');
axis([0 500 0 75]);






