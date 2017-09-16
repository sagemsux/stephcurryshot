function [c,ceq] = fminconNLConstraint(x) %This version consider friction effect of the wrist
[xe, ye, m1, m2, m3, l1, lc1, l2, lc2, l3, lc3, I2, I3, g, wx, wy] = getParameters();

%State-Variables
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
%Define constraint variable
%vx = qd2 * (l2 * sin(q2) + l3 * sin(q2+q3)) - qd3 * l3* sin(q2+q3) + 0.8;
%vy = qd1 + qd2 * (l2 * cos(q2) + l3 * cos(q2 + q3)) + qd3 * l3*cos(q2+q3) + 0.8;
%t = (xe - xr) / vx; % Time for ball to get to the rim
%Equality Constriant: Shooting Goal
%ceq(1) = vy * t - 0.5 * g * (t^2) - (ye - yr);
%Kinematics Constrain
ceq(1) = yr - q1 - l2 * sin(q2) - l3 * sin(q2+q3);
ceq(2) = xr - l2 * cos(q2) - l3 * cos(q2+q3);
%Equality Constraint: Dynamic Model
%{
ceq(2) = qdd2*(l2*m3*sin(q2) + lc2*m2*sin(q2) + lc3*m3*sin(q2 + q3)) + g*(m1 + m2 + m3) + qdd1*(m1 + m2 + m3) + lc3*m3*qdd3*sin(q2 + q3) + lc3*m3*cos(qd2 + qd3)*(qd2 + qd3)^2 + l2*m3*qd2^2*cos(q2) + lc2*m2*qd2^2*cos(q2) - tou1;
ceq(3) = - l2*lc3*m2*sin(q3)*qd3^2 - 2*l2*lc3*m3*qd2*sin(q3)*qd3 + qdd2*(m3*l2^2 + 2*m3*cos(q3)*l2*lc3 + m2*lc2^2 + m3*lc3^2 + I2 + I3) + g*(l2*m3*sin(q2) + lc2*m2*sin(q2) + lc3*m3*sin(q2 + q3)) + qdd1*(l2*m3*sin(q2) + lc2*m2*sin(q2) + lc3*m3*sin(q2 + q3)) + qdd3*(m3*lc3^2 + l2*m3*cos(q3)*lc3 + I3) - tou2;
ceq(4) = l2*lc3*m3*sin(q3)*qd2^2 + qdd3*(m3*lc3^2 + I3) + qdd2*(m3*lc3^2 + l2*m3*cos(q3)*lc3 + I3) + g*lc3*m3*sin(q2 + q3) + lc3*m3*qdd1*sin(q2 + q3) - tou3;
%}
% Upper hand shooting guesture
c(1) = qd2;
c(2) = -qd3;
%Shooting Goal
c(3) = abs(qd1 - qd1_ref) - 1e-3;
c(4) = abs(qd2 - qd2_ref) - 1e-3;
c(5) = abs(qd1 - qd3_ref) - 1e-3;
%c(3) = -vx;
%c(4) = -vy;
end