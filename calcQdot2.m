function [qd1, qd2, qd3] = calcQdot2(a, theta, ur, vr)
Jv = [ 0, 0.5*(a(1)*sin(theta(1))+a(2)*sin(theta(1)+theta(2))), -a(2)*sin(theta(1)+theta(2));
       0, 0, 0;
       1, a(1)*cos(theta(1))+a(2)*cos(theta(1)+theta(2)),  a(2)*cos(theta(1)+theta(2))];

w = pinv(Jv)*[ur - 0.8; 0 ; vr - 0.8];
qd1 = w(1);
qd2 = w(2);
qd3 = w(3);