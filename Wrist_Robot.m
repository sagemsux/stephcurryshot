clear; clc; close all
%Target Description
xe = 7.239; %m
ye = 3.048; %m
%Release Point - Steph's Release Point
xr = 0.372; %m
yr = 2.072; %m

%link configuration
q1 = 1.68;
wrist_angle = 2*pi/3;
palm_length = 4.26*2.54/100; %m 
xr = xr + palm_length*cos(wrist_angle); %Release Point adjusted by palm
yr = yr + palm_length*sin(wrist_angle);
l2 = 0.4033;
l3 = 0.4033;
l4 = palm_length; %Palm length
theta = 50/180*pi; %radians Optimal Distance

[ur, vr] = calcVelocity(xr, yr, xe, ye, theta);
[q2, q3, q4, check] = calcQ(q1, wrist_angle, l2, l3, l4, xr, yr);
[qd1,qd2,qd3,qd4] = calcQdot([l2, l3, l4], [q2, q3, q4], ur, vr);
% Simulation
ic = [1.421553 0 -1.8235 0 1.7949 0 pi/2 0];
fc = [q1 qd1 q2 qd2 q3 qd3 q4 qd4];
ts = 0; tf = 0.4;
t = linspace(0,0.4,100).';
b = [[ic(1) ic(3) ic(5) ic(7)];[fc(1) fc(3) fc(5) fc(7)];[ic(2) ic(4) ic(6) ic(8)];[fc(2) fc(4) fc(6) fc(8)]];
A = [ts^3    ts^2  ts  1;
    tf^3    tf^2  tf  1;
    3*ts^2  2*ts  1   0;
    3*tf^2  2*tf  1   0];
par = A\b;
qd = par(1,:).*t.^3 + par(2,:).*t.^2 + par(3,:).*t + par(4,:);
% Acceleration Phase
figure

for i = 1:length(t)    
    DH_table = [0, pi/2, qd(i,1), 0;
                l2, 0, 0, qd(i, 2);
                l3, 0, 0, qd(i, 3);
                l4, 0, 0, qd(i,4)];
    o1 = calcOi(DH_table,1);
    o2 = calcOi(DH_table,2);
    o3 = calcOi(DH_table,3);
    o4 = calcOi(DH_table,4);
    hold off
    plot([0 o1(1) o2(1) o3(1) o4(1)], [0 o1(3) o2(3) o3(3) o4(3)], 'LineWidth', 3)
    xlim([-0.5, 10]);
    ylim([-0.5, 6]);
    pause(0.05)
    %pause(2000* ts/length(t))
    hold on
end
plot(xe, ye, 'ro', 'MarkerSize', 10);
plot(xr, yr, 'bx');
quiver(xr, yr, ur, vr, 0.5)
g = 9.81;
tf = (xe - xr) / ur;
t = 0:0.05:tf;
x = o4(1) + ur*t;
y = o4(3) + vr*t - 0.5 * g * t.^2;
u = ur * ones(size(t));
v = vr - g*t;


for i = 1:length(t)
    hold on
    plot(x(i), y(i),'ko');
    %quiver(x(i), y(i), u(i), v(i),'b');
    pause(tf/length(t));
end