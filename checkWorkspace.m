function reacheable = checkWorkspace(xr, yr,h_min, h_max, l2, l3)
L = l2 + l3; %Maximum radius of link 2 and link 3
if yr > h_min && yr < h_max
    reacheable =  abs(xr) < L;
elseif yr > h_max
    reacheable = xr^2+(yr-h_max)^2 < L^2;
else %yr < h_min
    reacheable = xr^2+(yr-h_min)^2 < L^2;
end