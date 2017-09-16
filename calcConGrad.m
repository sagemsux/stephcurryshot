function [energyChange,motionChange,N] = calcConGrad(q1,q2,q3,qd1,qd2,qd3,E)


    %Steph Curry Motion
    
    t = linspace(0,0.4,100);
    
    steph_q1 = -11.495*t.^3 + 11.222*t.^2 - 1.6623*t + 1.4361;

    steph_q2 = 8.2662*t.^2 + 2.5581*t - 1.8489;

    steph_q3 = -1351.3*t.^4 + 1048*t.^3 - 264.14*t.^2 + 24.55*t + 1.2291;
    
    steph_E = 530;
    
    %Motion Planning for new point
    
    q0 = [1.421553, -1.8235, 1.7949]; %Initial Conditions
    qd0 = [0, 0, 0];
    
    coef_1 = traj_coefficient(q0(1), qd0(1), q1, qd1, t);
    coef_2 = traj_coefficient(q0(2), qd0(2), q2, qd2, t);
    coef_3 = traj_coefficient(q0(3), qd0(3), q3, qd3, t);
    traj_1 = coef_1(1) + coef_1(2)*t + coef_1(3) * t.^2 + coef_1(4) * t.^3;
    traj_2 = coef_2(1) + coef_2(2)*t + coef_2(3) * t.^2 + coef_2(4) * t.^3;
    traj_3 = coef_3(1) + coef_3(2)*t + coef_3(3) * t.^2 + coef_3(4) * t.^3;
    
    %Computing Adjustment
    
    adj_q1 = 0;
    adj_q2 = 0;
    adj_q3 = 0;
    adq1 = zeros(100);
    adq2 = zeros(100);
    adq3 = zeros(100);
    
    for i=1:length(t)
        
        adj_q1 = adj_q1 + abs(traj_1(i) - steph_q1(i))/abs(steph_q1(i));
        adj_q2 = adj_q2 + abs(traj_2(i) - steph_q2(i))/abs(steph_q2(i));
        adj_q3 = adj_q3 + abs(traj_3(i) - steph_q3(i))/abs(steph_q2(i));
        adq1(i) = abs(traj_1(i) - steph_q1(i))/abs(steph_q1(i));
        adq2(i) = abs(traj_2(i) - steph_q2(i))/abs(steph_q2(i));
        adq3(i) = abs(traj_3(i) - steph_q3(i))/abs(steph_q3(i));
    end
    
    total_adj = (adj_q1 + adj_q2 + adj_q3)/length(t);
    
    energyChange = (steph_E - E)/steph_E;
    
    motionChange = total_adj;
    
    N1 = norm(adq1);
    N2 = norm(adq2);
    N3 = norm(adq3);
    
    N = N1+N2+N3;
    
end