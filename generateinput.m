function [a, gamma, vf_new, wf_new, thetaf_new] = generateinput(del_t,n2,n1)
t = del_t;
xf = n2.coord(1);
yf = n2.coord(2);
thetaf = n2.theta;
vf = n2.v;
wf = n2.w;

xi = n1.coord(1);
yi = n1.coord(2);
thetai = n1.theta;
vi = n1.v;
wi = n1.w;

del_x = xf-xi;
del_y = yf-yi;

% calculate required gamma, steering acceleration
gamma = (1/(t^2))*(atan(del_y/del_x) - thetai - wi*t);

% calculate required a, translational acceleration
a = del_x/(cos(thetai + wi*t + gamma*(t^2))*(t^2)) - (vi/t);

% NOTE: 2-point BVD not solved since this input does not take robot to
% desired configuration's thetaf,vf,wf. Just takes it to the desired xf,yf
% in the given time interval and lets thetaf, vf, wf be whatever.

vf_new = vi + a*t;
wf_new = wi + gamma*t;
thetaf_new = thetai + wi*t + gamma*(t^2);
thetaf_new = (pi/180)*wrapTo360(thetaf_new*(180/pi));   % wraps theta to valid range of 0 to 2pi
end