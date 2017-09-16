function coef = traj_coefficient(q0, qd0, q1, qd1, t)
t0 = t(1);
tf = t(end);
poly = [1 t0 t0^2   t0^3;
        0  1 2*t0 3*t0^2;
        1 tf tf^2   tf^3;
        0  1 2*tf 3*tf^2;];
 
coef = eye(4) / poly * [q0 qd0 q1 qd1]';

%interpolation = @(t) coef(1)+coef(2)*t-coef(3)*t.^2+coef(4)*t.^3;
%traj = interpolation(t);