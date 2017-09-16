function [qd1,qd2,qd3] = CalcQdot(a, theta, ur, vr)
Jv = [ 0, -a(1)*sin(theta(1))-a(2)*sin(theta(1)+theta(2)), -a(2)*sin(theta(1)+theta(2));
       0, 0, 0;
       1, a(1)*cos(theta(1))+a(2)*cos(theta(1)+theta(2)),  a(2)*cos(theta(1)+theta(2))];

w = pinv(Jv)*[ur; 0 ; vr];

qd1 = w(1);
qd2 = w(2);
qd3 = w(3);