clear; clc; close all;

%Robot configuration

g = 9.81;

q0 = [1.421553, -1.8235, 1.7949];
qd0 = [0, 0, 0];

%Target Description
xe = 7.239; %m
ye = 3.048; %m

%Weights
m1 = 34.115;
m2 = 2.63;
m3 = 2.22;

%Length of Links

l2 = 0.3233;
lc2 = 0.16;

l3 = 0.4033;
lc3 = 0.25;

%Moments of Interia
I2 = 0.0916;
I3 = 0.1204;

%Release Point - Steph's Release Point
xr_actual = 0.31; %m
yr_actual = 2.18; %m

xr = linspace(0,l2+l3,200);
yr = linspace(1.67894,2.42824+l2+l3,200);

q1 = linspace(1.67894,2.42824,20);
%}
%{
xr = linspace(0,.182,2);
yr = linspace(1.67894,2.372,2);

q1 = linspace(1.67894,1.825,2);
%}

table = zeros(size(xr,2)*size(yr,2)*size(q1,2),9);

count = 1;

for i=1:size(xr,2)
    
    for j=1:size(yr,2)
        
        feasibility = checkWorkspace(xr(i),yr(j),q1(1),q1(size(q1,2)),l2,l3);
        if feasibility == false
            continue
        end
        
        [Vx, Vy] = calcVelocity(xr(i), yr(j), xe, ye, 50);
        
        for k=1:size(q1,2)
            
            [q2, q3, feasibility2] = calcQ(q1(k), l2, l3, xr(i), yr(j));
            
            if feasibility2 == false
                continue
            end
            
            [qd1, qd2, qd3] = calcQdot2([l2 l3],[q2 q3], Vx, Vy);
            
            [E,feasibility3] = calcMotionandEnergy([1.421553 0 -1.8235 0 1.7949 0],[q1(k) qd1 q2 qd2 q3 qd3]);
            
            if feasibility3 == false
                continue
            end
            
            table(count,:) = [xr(i) yr(j) q1(k) q2 q3 qd1 qd2 qd3 E];
            
            count = count + 1;
            
        end
    end
end

save('table.mat','table')

