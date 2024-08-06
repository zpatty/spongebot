%% 4D Arm Trajectory

  % "k1"  : 0.0,
  % "k2"  : 0.0,
  % "k3"  : 0.0,
  % "k4"  : 0.0,
  % "phi1": 0, 
  % "phi2": 0, 
  % "phi3": 0,
  % "phi4": 0,
  % "dL1" : -0.11,
  % "dL2" : -0.04,
  % "dL3" : -0.00,
  % "dL4" : -0.00,
  % "jm0" : 0.0,
  % "jm1" : 0.8,
  % "jm2" : 0.8,
  % "jm3" : 0.8

d = 0.04337;
theta = [0 0 0 0 0 0 0 0;
        0 0 0 0.5 0 0 0.4 0;
        0 0 0 1.3 0 0 1.3 0;
        0 0 0 1.3 0 0 1.3 0];
phi = [0 0 0 pi/2 0 0 0 0;
        0 0 0 pi/2 0 0 0 0;
        0 0 0 pi/2 0 0 pi 0;
        0 0 0 pi/2 0 0 pi 0];
dL = [-0.04 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 0.0;
    -0.03 -0.04 -0.04 -0.04 -0.09 -0.09 -0.05 -0.0;
    -0.0 -0.0 -0.0 -0.0 -0.04 -0.09 0.0 0.0;
    -0.0 -0.0 -0.0 -0.00 -0.04 -0.09 0.0 0.0];

dx = theta.*d.*cos(phi);
dy = theta.*d.*sin(phi);

motor = [0 0.5 1 1.5 1 0.5 0 0;
        -1.2 0.0 0.8 0.8 0.8 -0.4 -0.9 -1.2;
        -0.5 0 0.8 0.8 0.8 0 0.9 0;
        -1.2 0 0.8 0.8 0.8 -0.8 -1 -1.2];

wayPoints = [motor(1,:);dx(1,:);dy(1,:);dL(1,:);motor(2,:);dx(2,:);dy(2,:);dL(2,:);motor(3,:);
    dx(3,:);dy(3,:);dL(3,:); motor(4,:); dx(4,:);dy(4,:);dL(4,:)];
timePoints = [0 3 14 23 27 35 40 48]/3;
tvec = 0:0.001:timePoints(end);
tSamples = length(tvec);
[qd,dqd,ddqd,pp] = cubicpolytraj(wayPoints,timePoints,tvec);

save("~/SRRA/qd.mat", "qd")
save("~/SRRA/dqd.mat", "dqd")
save("~/SRRA/ddqd.mat", "ddqd")
save("~/SRRA/tvec.mat", "tvec")



%% for paper as of now

  % "k1"  : 0.0,
  % "k2"  : 0.0,
  % "k3"  : 0.0,
  % "k4"  : 0.0,
  % "phi1": 0, 
  % "phi2": 0, 
  % "phi3": 0,
  % "phi4": 0,
  % "dL1" : -0.11,
  % "dL2" : -0.04,
  % "dL3" : -0.00,
  % "dL4" : -0.00,
  % "jm0" : 0.0,
  % "jm1" : 0.8,
  % "jm2" : 0.8,
  % "jm3" : 0.8

d = 0.04337;
theta = [0 0 0 0 0 0 0 0;
        0 0 0 0.5 0 0 0.4 0;
        0 0 0 1.3 0 0 1.3 0;
        0 0 0 1.3 0 0 1.3 0];
phi = [0 0 0 pi/2 0 0 0 0;
        0 0 0 pi/2 0 0 0 0;
        0 0 0 pi/2 0 0 pi 0;
        0 0 0 pi/2 0 0 pi 0];
dL = [-0.04 -0.1 -0.1 -0.1 -0.1 -0.1 -0.1 0.0;
    -0.03 -0.04 -0.04 -0.04 -0.09 -0.09 -0.05 -0.0;
    -0.0 -0.0 -0.0 -0.0 -0.04 -0.09 0.0 0.0;
    -0.0 -0.0 -0.0 -0.00 -0.04 -0.09 0.0 0.0];

dx = theta.*d.*cos(phi);
dy = theta.*d.*sin(phi);

motor = [0 0.5 1 1.5 1 0.5 0 0;
        -1.2 0.0 0.8 0.8 0.8 -0.4 -0.9 -1.2;
        -0.5 0 0.8 0.8 0.8 0 0.9 0;
        -1.2 0 0.8 0.8 0.8 -0.8 -1 -1.2];

wayPoints = [motor(1,:);dx(1,:);dy(1,:);dL(1,:);motor(2,:);dx(2,:);dy(2,:);dL(2,:);motor(3,:);
    dx(3,:);dy(3,:);dL(3,:); motor(4,:); dx(4,:);dy(4,:);dL(4,:)];
timePoints = [0 3 8 12 25 35 40 48]/3;
tvec = 0:0.001:timePoints(end);
tSamples = length(tvec);
[qd,dqd,ddqd,pp] = cubicpolytraj(wayPoints,timePoints,tvec);

save("~/SRRA/qd.mat", "qd")
save("~/SRRA/dqd.mat", "dqd")
save("~/SRRA/ddqd.mat", "ddqd")
save("~/SRRA/tvec.mat", "tvec")







