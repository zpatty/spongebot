% bottom case

d = 0.04337;
% theta = [0 0 0 0 0 0 0 0 0;
%         0 0 0 1.5 1.5 1.5 1.5 1.5 1.5;
%         0 0 0 1.5 1.5 1.5 1.5 1.5 1.5;
%         0 0 0 1.5 1.5 1.5 0.0 0.0 1.0];
% phi = [0 0 0 pi/2 0 0 0 0 0;
%         0 0 0 pi pi pi pi pi pi;
%         0 0 0 pi pi pi pi 1.6 1.6;
%         0 0 0 pi pi pi 1.6 1.6 1.6];
% dL = [-0.04 -0.11 -0.11 -0.11 -0.11 -0.11 -0.11 -0.11 -0.11;
%     -0.03 -0.04 -0.06 -0.08 -0.03 -0.03 -0.03 -0.03 -0.03;
%     -0.0 -0.0 -0.0 -0.0 -0.04 -0.04 -0.04 -0.04 -0.04;
%     -0.0 -0.0 -0.0 -0.00 -0.04 -0.04 -0.04 -0.04 -0.04];

%% TODO: make third module slightyl less curvy
theta = [0 0 0 0 0 0 0 0 0 0;
        0 0 0 1.5 1.5 1.5 1.5 0 1.5 1.7;
        0 0 0 1.5 1.5 1.5 0 1.5 1.5 1.7;
        0 0 0 0.7 1.5 1.0 0.5 1.0 1.5 1.7];
phi = [0 0 0 0 0 0 0 0 0 0;
        0 0 0 3.5 3.5 3.5 3.5 1.6 1.8 1.8;
        0 0 0 3.5 3.5 3.5 3.5 1.6 1.6 1.6;
        0 0 0 1.6 1.6 1.6 1.6 1.6 1.6 1.6];
dL = [-0.04 -0.11 -0.11 -0.11 -0.11 -0.11 -0.11 -0.11 -0.11 -0.11;
    -0.03 -0.04 -0.06 -0.06 -0.06 -0.06 -0.06 -0.11 -0.05 -0.03;
    -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0;
    -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0 -0.0];
dx = theta.*d.*cos(phi);
dy = theta.*d.*sin(phi);

motor = [0 -1.1 -1.1 -1.1 -1.35 -1.55 -1.1 -0.85 -0.5 -1.0;
        -1.2 0 0.8 0.8 0.5 0.10 -0.2 -0.7 -0.9 -0.9;
        -0.5 0 -1.2 -1.2 -1.1 -0.95 -0.95 -1.1 -0.3 -0.0;
        -1.2 0 0.0 0.0 0.0 0.4 0.0 0.0 0.4 0.4];
wayPoints = [motor(1,:);dx(1,:);dy(1,:);dL(1,:);motor(2,:);dx(2,:);dy(2,:);dL(2,:);motor(3,:);
    dx(3,:);dy(3,:);dL(3,:); motor(4,:); dx(4,:);dy(4,:);dL(4,:)];
% timePoints = [0 3 8 10 13 15 18 21 25];
timePoints = [0 3 6 8 12 14 16 20 24 27];
tvec = 0:0.005:30;
tSamples = length(tvec);
[qd,dqd,ddqd,pp] = cubicpolytraj(wayPoints,timePoints,tvec);

save("first_plate/qd.mat", "qd")
save("first_plate/dqd.mat", "dqd")
save("first_plate/ddqd.mat", "ddqd")
save("first_plate/tvec.mat", "tvec")