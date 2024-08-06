%% 4D Arm Trajectory
d = 0.04337;
theta = [0 0.0 0.0 -0.0;
        0 0.0 0.0 0.0;
        0 0 0 0;
        0 0.0 0.0 0.0];
phi = [0 0 0 0;
        0 0 0 0;
        0 0 0 0;
        0 0 0 0];
dL = [-0.0 -0.03 -0.03 -0.03;
    -0.0 -0.0 -0.0 -0.0;
    -0.0 -0.0 -0.0 -0.0
    -0.0 -0.0 -0.0 -0.0;];
dL = [-0.2 -0.2 -0.2 -0.2;
    -0.13 -0.13 -0.13 -0.13;
    -0.13 -0.13 -0.13 -0.13
    -0.13 -0.13 -0.13 -0.13;];

dx = theta.*d.*cos(phi);
dy = theta.*d.*sin(phi);

motor = [0.3 0.3 -pi/2 -pi/2;
        70*pi/180 70*pi/180 70*pi/180 70*pi/180;
        70*pi/180 70*pi/180 70*pi/180 70*pi/180;
        70*pi/180 70*pi/180 70*pi/180 70*pi/180];
motor = [0.3 0.3 -pi/2 -pi/2;
        70*pi/180*ones(1,4);
        0.9*ones(1,4);
        zeros(1,4)];

wayPoints = [motor(1,:);dx(1,:);dy(1,:);dL(1,:);motor(2,:);dx(2,:);dy(2,:);dL(2,:);motor(3,:);
    dx(3,:);dy(3,:);dL(3,:);motor(4,:);dx(4,:);dy(4,:);dL(4,:)];
timePoints = [0 10 12 13];
tvec = 0:0.01:13;
tSamples = length(tvec);
[qd,dqd,ddqd,pp] = cubicpolytraj(wayPoints,timePoints,tvec);

save("~/SRRA/qd.mat", "qd")
save("~/SRRA/dqd.mat", "dqd")
save("~/SRRA/ddqd.mat", "ddqd")
save("~/SRRA/tvec.mat", "tvec")



