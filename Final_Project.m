% Final Project
clear all;

global L1 L2 O1 O2 a1 a2 c d psi mu a theta wr eJtheta
L1 = 0.8; % Length of the first link
L2 = 0.5; % Length of the second link
O1 = -0.666; % Coordinates of the first robot base in the work-space frame
O2 = -0.333; % Coordinates of the second robot base in the work-space frame

% Parameters for tracking trajectory
% yd dot = - mu yd + r
mu = 1;

% Parameters for calculating the reference signal
% r1(t) = a1 sin(wr t) + c + d sin(1.5 wr t)
% r2(t) = a2 sin(wr t + psi) + c + d sin(1.5 wr t + psi)
a1 = 0.04; 
a2 = 0.04;
d = 0.04;
c = 0.1;
psi = 1;
wr = 0.25;

% Transformation for the desired reference trajectory
% xd = a e^{J theta} yd
a = 1.3;
theta = pi/6;
J = [0, -1; 1, 0];
eJtheta = [cos(theta), -sin(theta); sin(theta), cos(theta)];

%% Calculate the reference signal
global dt
t0 = 0;
tf = 50;
dt = 0.01;

N = floor((tf - t0)/dt); % number of steps

yd0 = [0; 0];
[t, yd] = rk4fixed('ReferenceTrajectory', [t0 tf], yd0, N);
x_ref = zeros(2, N);
q_ref = zeros(2, N);
for i=1:N
    [yddot, y] = ReferenceTrajectory(t(i), yd(i,:));
    x_ref(1, i) = y(3);
    x_ref(2, i) = y(4);
    q_ref(1, i) = y(5);
    q_ref(2, i) = y(6);
end

figure(1);
title('Reference trajectory');
plot(t, q_ref(1, :), t, q_ref(2, :));
legend('q_1', 'q_2');
xlabel('Time (s)');
ylabel('Angle (rad)');

%% Initial state
q0 = [1.3; -1.3];
qdot0 = [0; 0];
yd0 = [0; 0];
x0 = [yd0; q0; qdot0];
%% Non-adaptive Control
global Kp p1 p2 p3 Phi;
Kp = [1, 0; 0, 1];
p1 = 3.31;
p2 = 0.116;
p3 = 0.16;
Phi = [p1; p2; p3];

[t, x] = rk4fixed('NonAdaptiveControl', [t0 tf], x0, N);
for i=1:N
    [xd, y] = NonAdaptiveControl(t(i), x(i,:));
    b_qd1(i) = y(1);
    b_qd2(i) = y(2);
    b_q1(i) = y(3);
    b_q2(i) = y(4);
    b_torque1(i) = y(5);
    b_torque2(i) = y(6);
end

figure(2);
title('Non-adaptive Control Tracking $q_1$','Interpreter','latex');
plot(t, b_qd1, t, b_q1);
legend('q_{d1}', 'q_1');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(3);
title('Non-adaptive Control Tracking q_2');
plot(t, b_qd2, t, b_q2);
legend('q_{d2}', 'q_2');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(4);
title('Non-adaptive Control Torque');
plot(t, b_torque1, t, b_torque2);
legend('\tau_1', '\tau_2');
xlabel('Time (s)');
ylabel('Torque (N \cdot m)');
%% Adaptive Control 
global Gamma Phihat Kp;
Kp = [1, 0; 0, 1];
Gamma = [0.1, 0, 0; 0, 0.1, 0; 0, 0, 0.1];
Phihat = [3.11; 0.216; 0.24];

[t, x] = rk4fixed('AdaptiveControl', [t0 tf], x0, N);
for i=1:N
    [xd, y] = AdaptiveControl(t(i), x(i,:));
    c_qd1(i) = y(1);
    c_qd2(i) = y(2);
    c_q1(i) = y(3);
    c_q2(i) = y(4);
    c_torque1(i) = y(5);
    c_torque2(i) = y(6);
    c_phi1(i) = y(7);
    c_phi2(i) = y(8);
    c_phi3(i) = y(9);
end

figure(5);
title('Adaptive Control Tracking $q_1$','Interpreter','latex');
plot(t, c_qd1, t, c_q1);
legend('q_{d1}', 'q_1');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(6);
title('Adaptive Control Tracking q_2');
plot(t, c_qd2, t, c_q2);
legend('q_{d2}', 'q_2');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(7);
title('Adaptive Control Torque');
plot(t, c_torque1, t, c_torque2);
legend('\tau_1', '\tau_2');
xlabel('Time (s)');
ylabel('Torque (N \cdot m)');
%% Non-smooth Projection Adaptive Control
global Phimin Phimax PhiBar sigma Kp;
Kp = [1, 0; 0, 1];
Gamma = [0.1, 0, 0; 0, 0.1, 0; 0, 0, 0.1];
Phihat = [3.11; 0.216; 0.24];
PhiBar = [3.11; 0.216; 0.24];
Phimin = [3.05; 0.05; 0.1];
Phimax = [3.35; 0.25; 0.25];
sigma = diag([0.1; 0.1; 0.1]);

[t, x] = rk4fixed('NonSmoothAdaptiveControl', [t0 tf], x0, N);
for i=1:N
    [xd, y] = NonSmoothAdaptiveControl(t(i), x(i,:));
    d_qd1(i) = y(1);
    d_qd2(i) = y(2);
    d_q1(i) = y(3);
    d_q2(i) = y(4);
    d_torque1(i) = y(5);
    d_torque2(i) = y(6);
    d_phi1(i) = y(7);
    d_phi2(i) = y(8);
    d_phi3(i) = y(9);
end

figure(8);
title('Non-smooth Projection Adaptive Control Tracking $q_1$','Interpreter','latex');
plot(t, d_qd1, t, d_q1);
legend('q_{d1}', 'q_1');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(9);
title('Non-smooth Projection Adaptive Control Tracking q_2');
plot(t, d_qd2, t, d_q2);
legend('q_{d2}', 'q_2');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(10);
title('Non-smooth Projection Adaptive Control Torque');
plot(t, d_torque1, t, d_torque2);
legend('\tau_1', '\tau_2');
xlabel('Time (s)');
ylabel('Torque (N \cdot m)');
%% Smooth Projection Adaptive Control
global Phimin Phimax Eta Kp;
Kp = [1, 0; 0, 1];
Eta = [0; 0; 0];
Gamma = [0.1, 0, 0; 0, 0.1, 0; 0, 0, 0.1];
Phihat = [3.11; 0.216; 0.24];
Phimin = [3.05; 0.05; 0.1];
Phimax = [3.35; 0.25; 0.25];

[t, x] = rk4fixed('SmoothAdaptiveControl', [t0 tf], x0, N);
for i=1:N
    [xd, y] = SmoothAdaptiveControl(t(i), x(i,:));
    e_qd1(i) = y(1);
    e_qd2(i) = y(2);
    e_q1(i) = y(3);
    e_q2(i) = y(4);
    e_torque1(i) = y(5);
    e_torque2(i) = y(6);
    e_phi1(i) = y(7);
    e_phi2(i) = y(8);
    e_phi3(i) = y(9);
end

figure(11);
title('Smooth Projection Adaptive Control Tracking $q_1$','Interpreter','latex');
plot(t, e_qd1, t, e_q1);
legend('q_{d1}', 'q_1');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(12);
title('Smooth Projection Adaptive Control Tracking q_2');
plot(t, e_qd2, t, e_q2);
legend('q_{d2}', 'q_2');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(13);
title('Smooth Projection Adaptive Control Torque');
plot(t, e_torque1, t, e_torque2);
legend('\tau_1', '\tau_2');
xlabel('Time (s)');
ylabel('Torque (N \cdot m)');

%% Comparation
figure(14);
title('Simulation Tracking Result of $q_1$');
plot(t, b_qd1, t, b_q1, t, c_q1, t, d_q1, t, e_q1);
legend('Reference Trajectory', 'Non-Adaptive Control', 'Adaptive Control', 'Non-smooth Projection Adaptive Control', 'Smooth Projection Adaptive Control');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(15);
title('Simulation Tracking Result of $q_2$');
plot(t, b_qd2, t, b_q2, t, c_q2, t, d_q2, t, e_q2);
legend('Reference Trajectory', 'Non-Adaptive Control', 'Adaptive Control', 'Non-smooth Projection Adaptive Control', 'Smooth Projection Adaptive Control');
xlabel('Time (s)');
ylabel('Angle (rad)');

figure(16);
title('Simulation Tracking Result of Torque 1');
plot(t, b_torque1, t, c_torque1, t, d_torque1, t, e_torque1);
legend('Non-Adaptive Control', 'Adaptive Control', 'Non-smooth Projection Adaptive Control', 'Smooth Projection Adaptive Control');
xlabel('Time (s)');
ylabel('Torque (N \cdot m)');

figure(17);
title('Simulation Tracking Result of Torque 2');
plot(t, b_torque2, t, c_torque2, t, d_torque2, t, e_torque2);
legend('Non-Adaptive Control', 'Adaptive Control', 'Non-smooth Projection Adaptive Control', 'Smooth Projection Adaptive Control');
xlabel('Time (s)');
ylabel('Torque (N \cdot m)');

figure(18);
title('Simulation Result of Parameter $p_1$');
plot(t, c_phi1, t, d_phi1, t, e_phi1);
legend('Adaptive Control', 'Non-smooth Projection Adaptive Control', 'Smooth Projection Adaptive Control');
xlabel('Time (s)');
ylabel('p_1 (kg \cdot m^2)');


figure(19);
title('Simulation Result of Parameter $p_2$');
plot(t, c_phi2, t, d_phi2, t, e_phi2);
legend('Adaptive Control', 'Non-smooth Projection Adaptive Control', 'Smooth Projection Adaptive Control');
xlabel('Time (s)');
ylabel('p_2 (kg \cdot m^2)');


figure(20);
title('Simulation Result of Parameter $p_3$');
plot(t, c_phi3, t, d_phi3, t, e_phi3);
legend('Adaptive Control', 'Non-smooth Projection Adaptive Control', 'Smooth Projection Adaptive Control');
xlabel('Time (s)');
ylabel('p_3 (kg \cdot m^2)');