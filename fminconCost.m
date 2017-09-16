function E = fminconCost(x)
t = linspace(0,0.4,100).';
[xe, ye, m1, m2, m3, l1, lc1, l2, lc2, l3, lc3, I2, I3, g, wx, wy] = getParameters();
fc = x(1:6);
ic = [1.421553 0 -1.8235 0 1.7949 0];
%Motion Planing
%ic = [0 0 0 0 0 0];
%fc = [q1f qd1f q2f qd2f q3f qd3f];
ts = 0; tf = 0.4;
b = [[ic(1) ic(3) ic(5)];[fc(1) fc(3) fc(5)];[ic(2) ic(4) ic(6)];[fc(2) fc(4) fc(6)]];
A = [ts^3    ts^2  ts  1;
    tf^3    tf^2  tf  1;
    3*ts^2  2*ts  1   0;
    3*tf^2  2*tf  1   0];
par = A\b;

qd = par(1,:).*t.^3 + par(2,:).*t.^2 + par(3,:).*t + par(4,:);
qd_dot = 3*par(1,:).*t.^2 + 2*par(2,:).*t + par(3,:);
qd_ddot = 6*par(1,:).*t + 2*par(2,:);

q1 = qd(:,1); q2 = qd(:,2)+pi/2; q3 = qd(:,3);
qd1 = qd_dot(:,1); qd2 = qd_dot(:,2); qd3 = qd_dot(:,3);
qdd1 = qd_ddot(:,1); qdd2 = qd_ddot(:,2); qdd3 = qd_ddot(:,3);

tau1 = (g*m1 + g*m2 + g*m3 + m1*qdd1 + (m3*(2*qdd1 + 2.*lc3*cos(q2 + q3).*(qd2 + qd3).^2 + 2*l2*qdd2.*sin(q2) + 2*l2*qd2.^2.*cos(q2) + 2*lc3*sin(q2 + q3).*(qdd2 + qdd3)))/2 + (m2*(2*lc2*cos(q2).*qd2.^2 + 2*qdd1 + 2*lc2*qdd2.*sin(q2)))/2);
tau2 = (I2*qdd2 + (I3*(2*qdd2 + 2*qdd3))/2 + m3*(lc3*cos(q2 + q3) + l2*cos(q2)).*(l2*qdd2.*cos(q2) - lc3*sin(q2 + q3).*(qd2 + qd3).^2 - l2*qd2.^2.*sin(q2) + lc3*cos(q2 + q3).*(qdd2 + qdd3)) - m3*(l2*qd2.*cos(q2) + lc3*cos(q2 + q3).*(qd2 + qd3)).*(l2*qd2.*sin(q2) + lc3*sin(q2 + q3).*(qd2 + qd3)) + m3*(lc3*sin(q2 + q3) + l2*sin(q2)).*(qdd1 + lc3*cos(q2 + q3).*(qd2 + qd3).^2 + l2*qdd2.*sin(q2) + l2*qd2.^2.*cos(q2) + lc3*sin(q2 + q3).*(qdd2 + qdd3)) + m3*(l2*qd2.*cos(q2) + lc3*cos(q2 + q3).*(qd2 + qd3)).*(qd1 + l2*qd2.*sin(q2) + lc3*sin(q2 + q3).*(qd2 + qd3)) + lc2^2*m2*qdd2.*cos(q2).^2 + g*lc3*m3*sin(q2 + q3) + lc2*m2*sin(q2).*(lc2*cos(q2).*qd2.^2 + qdd1 + lc2*qdd2.*sin(q2)) + g*l2*m3*sin(q2) + g*lc2*m2*sin(q2) - 2*lc2^2*m2*qd2.^2.*cos(q2).*sin(q2) + lc2*m2*qd2.*cos(q2).*(qd1 + lc2*qd2.*sin(q2)) - lc3*m3*qd1.*qd2.*cos(q2 + q3) - lc3*m3*qd1.*qd3.*cos(q2 + q3) - l2*m3*qd1.*qd2.*cos(q2) - lc2*m2*qd1.*qd2.*cos(q2));
tau3 = ((I3*(2*qdd2 + 2*qdd3))/2 + lc3*m3*sin(q2 + q3).*(qdd1 + lc3*cos(q2 + q3).*(qd2 + qd3).^2 + l2*qdd2.*sin(q2) + l2*qd2.^2.*cos(q2) + lc3*sin(q2 + q3).*(qdd2 + qdd3)) + g*lc3*m3*sin(q2 + q3) + lc3*m3*cos(q2 + q3).*(l2*qdd2.*cos(q2) - lc3*sin(q2 + q3).*(qd2 + qd3).^2 - l2*qd2.^2.*sin(q2) + lc3*cos(q2 + q3).*(qdd2 + qdd3)) - lc3*m3*sin(q2 + q3).*(l2*qd2.*cos(q2) + lc3*cos(q2 + q3).*(qd2 + qd3)).*(qd2 + qd3) - lc3*m3*qd1.*qd2.*cos(q2 + q3) - lc3*m3*qd1.*qd3.*cos(q2 + q3) + lc3*m3*cos(q2 + q3).*(qd2 + qd3).*(qd1 + l2*qd2.*sin(q2) + lc3*sin(q2 + q3).*(qd2 + qd3)) - l2*lc3*m3*qd2.^2.*cos(q2 + q3).*sin(q2) + l2*lc3*m3*qd2.^2.*sin(q2 + q3).*cos(q2) - l2*lc3*m3*qd2.*qd3.*cos(q2 + q3).*sin(q2) + l2*lc3*m3*qd2.*qd3.*sin(q2 + q3).*cos(q2));

if tau1 < - 600 | tau1 > 600
    E = 1e6;
elseif tau2 < -25 | tau2 > 45
    E = 1e6;
elseif tau3 < -10 | tau3 > 60
    E = 1e6;
else
    e1 = 0; e2 = 0; e3 = 0;
    for i = 2:size(tau1,1)
        e1 = e1 + abs(tau1(i)*(q1(i)-q1(i-1)));
        e2 = e2 + abs(tau2(i)*(q2(i)-q2(i-1)));
        e3 = e3 + abs(tau3(i)*(q3(i)-q3(i-1)));
    end
    E = (e1 + e2 + e3);
end
    