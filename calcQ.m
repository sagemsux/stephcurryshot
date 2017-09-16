function  [q2, q3, check] = calcQ(q1, a2, a3, xr, yr)
check = true;
try
    H = yr - q1;
    C2 = (xr^2 + H^2 - a2^2 - a3^2) / (2*a2*a3);
    q3 = atan2(sqrt(1-C2^2), C2);
    alpha = atan2(H, xr);
    beta = atan2(a3*sin(q3), a2 + a3*cos(q3));
    q2 = alpha - beta;
    
    if q2 < -pi/2 || q2 > 40*pi/180
        check = false;
    end
    
    if q3 < 10*pi/180 || q3 > 100*pi/180
        check = false;
    end
    
catch
    check = false;
    q2 = 0;
    q3 = 0;
end
end