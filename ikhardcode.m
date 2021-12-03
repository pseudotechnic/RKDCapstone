b11app = [1.0850 0.8181 1.6279 0.8550 0.4154-pi/4]';

b11plc = [1.0850 0.7616 1.6279 0.8550 0.4154-pi/4]';

b12app = [1.0372 0.8494 1.7497 0.8648 0.4154-pi/4]';

b12plc = [1.0372 0.7894 1.7497 0.8648 0.4154-pi/4]';

b13app = [1.0108 0.8996 1.7778 0.9007 0.4153-pi/4]';

b13plc = [1.0108 0.8496 1.7778 0.9007 0.4153-pi/4]';

b21app = [0.9904 0.7621 1.6075 0.9294 0.4154-3*pi/4]';
b21plc = [0.9904 0.7222 1.5902 0.9273 0.4154-3*pi/4]';
b22app = [0.9646 0.7856 1.6391 0.8665 0.4154-3*pi/4]';
b22plc = [0.9646 0.7414 1.6363 0.9213 0.4154-3*pi/4]';
b23app = [0.9272 0.8023 1.7015 0.9194 0.4153-3*pi/4]';
b23plc = [0.9272 0.7618 1.7026 0.9744 0.4154-3*pi/4]';

b31app = [0.9904 0.7621 1.6075 0.9294 0.4154-pi/4]';
b31plc = [0.9904 0.7222 1.5902 0.9273 0.4154-pi/4]';
b32app = [0.9646 0.7856 1.6391 0.8665 0.4154-pi/4]';
b32plc = [0.9646 0.7414 1.6363 0.9213 0.4154-pi/4]';
b33app = [0.9272 0.8023 1.7015 0.9194 0.4153-pi/4]';
b33plc = [0.9272 0.7618 1.7026 0.9744 0.4154-pi/4]';

b41app = [0.9904 0.7621 1.6075 0.9294 0.4154-3*pi/4]';
b41plc = [0.9904 0.7222 1.5902 0.9273 0.4154-3*pi/4]';
b42app = [0.9646 0.7856 1.6391 0.8665 0.4154-3*pi/4]';
b42plc = [0.9646 0.7414 1.6363 0.9213 0.4154-3*pi/4]';
b43app = [0.9272 0.8023 1.7015 0.9194 0.4153-3*pi/4]';
b43plc = [0.9272 0.7618 1.7026 0.9744 0.4154-3*pi/4]';

b51app = [0.9904 0.7621 1.6075 0.9294 0.4154-pi/4]';
b51plc = [0.9904 0.7222 1.5902 0.9273 0.4154-pi/4]';
b52app = [0.9646 0.7856 1.6391 0.8665 0.4154-pi/4]';
b52plc = [0.9646 0.7414 1.6363 0.9213 0.4154-pi/4]';
b53app = [0.9272 0.8023 1.7015 0.9194 0.4153-pi/4]';
b53plc = [0.9272 0.7618 1.7026 0.9744 0.4154-pi/4]';

b61app = [0.9904 0.7621 1.6075 0.9294 0.4154-3*pi/4]';
b61plc = [0.9904 0.7222 1.5902 0.9273 0.4154-3*pi/4]';
b62app = [0.9646 0.7856 1.6391 0.8665 0.4154-3*pi/4]';
b62plc = [0.9646 0.7414 1.6363 0.9213 0.4154-3*pi/4]';
b63app = [0.9272 0.8023 1.7015 0.9194 0.4153-3*pi/4]';
b63plc = [0.9272 0.7618 1.7026 0.9744 0.4154-3*pi/4]';

dropapp = [b11app b12app b13app b21app b22app b23app b31app b32app b33app];
drop = [b11plc b12plc b13plc b21plc b22plc b23plc b31plc b32plc b33plc];

for i=1:3
    q = drop(:,i);
    
    theta0 = rad2deg(q(1));
    theta1 = rad2deg(q(2));
    big2 = rad2deg(q(3));

    arm0 = 70.5;
    arm1 = 381;
    theta2 = big2 - theta1;
    arm2 = 317.5;
    theta3 = big2 - theta1;
    arm4 = 55;
    eeoff = 15;

    distfrombase = cosd(theta1) * arm1 + sind(theta2) * arm2 + eeoff;
    height = arm0 + (sind(theta1) * arm1) - (cosd(theta2) * arm2) - arm4;

    disp(["angle", "dist", "height", i]);
    disp([theta0, distfrombase, height]);
end

%%IK

%xs = [527.7 500 482.6 482.6 500 527.05 527.7 500 482.6];% 482.6 500 527.05 527.7 500 482.6 482.6 500 527.05];
xs = [532.6575];
ys = [72.7917];
%ys = [25, 25, 25, 40, 40, 40, 55, 55, 55]; %[20, 20, 20, 35, 35, 35, 50, 50, 50];

l1 = 381;
l2 = 317.5;
%disp(size(xs, 2));
for i=1:size(xs,2)
    psi2 = acos(((xs(i)^2 + ys(i)^2) - l1^2 - l2^2)/(2 * l1 * l2));
    psi1 = atan2(ys(i),xs(i));% - atan2(l2 * sin(psi2), l1 + l2 * cos(psi2));
    psi1 = psi1 + pi/2;
    disp(["theta3","theta2","wp#"])
    disp([psi1, psi2, i]);
end