clear; clc; close all;
%Release Point - Steph's Release Point
%xr_actual = 0.31; %m
%yr_actual = 2.18; %m
xr = 0.31; %m
yr = 1.98;%2.18; %m
%q0 = [1.421553, -1.8235, 1.7949]; %Steph's releasing configuration

%Initial Guess for fmincon
[xe, ye, m1, m2, m3, l1, lc1, l2, lc2, l3, lc3, I2, I3, g, wx, wy] = getParameters();
x0 =  [1.8277, 3.2293, -4.0816, 0.5054, 11.9441, 52.3612, 1.8235, 7.9030, 69.2550, 323.5486, 12.1785,59.5487]; %Default Configuration
q1 = 1.421553; %m
[q2, q3, check] = calcQ(q1, l2, l3, xr, yr);
%{
%Verify Kinematics solution
yr - q1 - l2 * sin(q2) - l3 * sin(q2+q3)
xr - l2 * cos(q2) - l3 * cos(q2+q3)
%}
%
[Vx, Vy] = calcVelocity(xr, yr, xe, ye, 50); %shooting velocity
[qd1, qd2, qd3] = calcQdot2([l2 l3],[q2 q3], Vx, Vy);
x0 = [q1; q2; q3; qd1; qd2; qd3; xr; yr];
%% Execute fmincon
A = [];
b = [];
Aeq = [];
beq = [];
lb = [1.4;%1.67894;
      0;
      10*pi/180;
      -Inf;
      -Inf;
      -Inf;
      0;
      1.67894];
ub = [2.42824;
      120*pi/180;
      100*pi/180;
      Inf;
      Inf;
      Inf;
      l2+l3;
      2.42824+l2+l3];

fun = @fminconCost;
nonlcon = @fminconNLConstraint;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations', 1600);

x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
%% Verification
q1 = x(1);
q2 = x(2);
q3 = x(3);
qd1 = x(4);
qd2 = x(5);
qd3 = x(6);
xr = x(7);
yr = x(8);

[Vx, Vy] = calcVelocity(xr, yr, xe, ye, 50);
[qd1_ref, qd2_ref, qd3_ref] = calcQdot2([l2 l3],[q2 q3], Vx, Vy);

eqn(1) = yr - q1 - l2 * sin(q2) - l3 * sin(q2+q3);
eqn(2) = xr - l2 * cos(q2) - l3 * cos(q2+q3);

ineqn(1) = qd2;
ineqn(2) = -qd3;
%Shooting Goal
ineqn(3) = abs(qd1 - qd1_ref) - 1e-3;
ineqn(4) = abs(qd2 - qd2_ref) - 1e-3;
ineqn(5) = abs(qd1 - qd3_ref) - 1e-3;

%% Brueforce soluvtion
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

