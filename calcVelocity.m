function [ur, vr] = calcVelocity(xr, yr, xe, ye, theta)
g = 9.81; % ms^-2
v0 = sqrt(0.5*g*(xe-xr)^2/(sind(theta)*cosd(theta)*(xe-xr)-(ye-yr)*(cosd(theta)^2)));
ur = v0 * cosd(theta);
vr = v0 * sind(theta);