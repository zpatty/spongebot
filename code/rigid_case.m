% current best traj
d = 0.04337;
theta = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
phi = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

%% TODO: get the second module to also bend with the top two
dL = [0 -0.06 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1;
        0 -0.04 -0.06 -0.1 -0.1 -0.1 -0.1 -0.1 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08;
        0 0 -0.0 -0.1 -0.1 -0.1 -0.1 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08;
        0 0 0 0 -0.0 -0.06 -0.1 -0.1 -0.09 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08 -0.08];

dx = theta.*d.*cos(phi);
dy = theta.*d.*sin(phi);

motor = [0 0 0 0 0 0 0 0 0 0 -1.6 -1.6 -1.6 -1.6 -1.6 -1.6 -1.6;
        -1.0 -0.8 -0.5 -0.2 -0.0 -0.0 -0.0 0.0 0.0 0.0 1.0 1.0 0.8 0.8 0.6 0.3 0.3;
        -0.4 -0.7 -1 -0.9 -0.7 -0.6 -0.4 0.2 -0.0 -0.3 -0.3 -0.6 -0.95 -0.95 -0.95 -0.95 -0.95;
        -1.0 -1.0 -1.0 -0.9 -0.7 -0.6 -0.4 -0.0 -0.4 -0.4 -0.7 -1.1 -0.6 -0.3 0.2 0.2 0.2];

wayPoints = [motor(1,:);dx(1,:);dy(1,:);dL(1,:);motor(2,:);dx(2,:);dy(2,:);dL(2,:);motor(3,:);
    dx(3,:);dy(3,:);dL(3,:); motor(4,:); dx(4,:);dy(4,:);dL(4,:)];
timePoints = [0 3 14 23 27 35 40 48 54 60 68 74 85 92 96 99 103]/7;
tvec = 0:0.001:timePoints(end);
tSamples = length(tvec);
[qd,dqd,ddqd,pp] = cubicpolytraj(wayPoints,timePoints,tvec);

plot(tvec,qd, 'LineWidth',2)
title('Trajectory (qd) over Time')
xlabel('time') 
ylabel('qd values') 
legend('j0','dx1', 'dy1', 'dL1', 'j1','dx2', 'dy2', 'dL2', 'j2', 'dx3', 'dy3','dL3', 'j3', 'dx4', 'dy4','dL4')
save("tube_up/qd.mat", "qd")
save("tube_up/dqd.mat", "dqd")
save("tube_up/ddqd.mat", "ddqd")
save("tube_up/tvec.mat", "tvec")