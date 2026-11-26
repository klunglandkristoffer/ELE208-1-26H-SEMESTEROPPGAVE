%% ELE208 - 42i3 Inverse Kinematics demo
clear; clc; close all;

% Robot dimensions [m] (same as a02_RobotModel_REV5)
L1 = 0.150; L2 = 1.060; L3 = 0.800; L4 = 0.263;
standoff = 0.1;

% Standard DH: Link([theta d a alpha])
J1 = Link([0 L1 0 pi/2 0], 'standard');
J2 = Link([0 standoff  L2 0 0], 'standard');
J3 = Link([0 -standoff L3 0 0], 'standard');
J4 = Link([0 standoff  L4 0 0], 'standard');
J2.offset = pi/2;
robot = SerialLink([J1 J2 J3 J4], 'name', 'ELE208_4DOF');

% Target: pickup point 1.0 m from base, cube centre on the belt
p_target = [1.0 0 0.025];

% Solve inverse kinematics (gripper pointing straight down, elbow up)
q = solve_ik(robot, p_target);
if isempty(q), error('Target unreachable.'); end
disp('Solution for joint angles [deg]:'); disp(rad2deg(q));

% Forward kinematics validation
T = robot.fkine(q).T;
disp('Target position [m]:'); disp(p_target);
disp('Computed pose from FK:'); disp(T);
fprintf('Position error: %.2e m\n', norm(T(1:3,4)' - p_target));
fprintf('Gripper axis (should be [0 0 -1]): %s\n', mat2str(T(1:3,1)', 3));

% Plot robot in the solved configuration
robot.plot(q, 'workspace', [-0.5 1.5 -1 1 0 1.5]);
hold on;
plot3(p_target(1), p_target(2), p_target(3), 'r*', 'MarkerSize', 12);

function q = solve_ik(robot, p)
    % Inverse kinematics, end-effector pointing straight down, elbow up.
    L1 = robot.d(1); L2 = robot.a(2); L3 = robot.a(3); L4 = robot.a(4);
    D  = sum(robot.d(2:4));                     % Sideways offset of the tip

    r = hypot(p(1), p(2));
    if r < abs(D), q = []; return; end
    rho = sqrt(r^2 - D^2);                      % Reach in the arm plane
    q1  = atan2(p(2), p(1)) + atan2(D, rho);
    q1  = atan2(sin(q1), cos(q1));

    hw = p(3) - L1 + L4;                        % Wrist height above shoulder
    c3 = (rho^2 + hw^2 - L2^2 - L3^2) / (2*L2*L3);
    if abs(c3) > 1, q = []; return; end
    q3  = -acos(c3);                            % Elbow up
    th2 = atan2(hw, rho) - atan2(L3*sin(q3), L2 + L3*cos(q3));
    q2  = th2 - pi/2;                           % DH offset on joint 2
    q4  = -pi/2 - th2 - q3;                     % Keep the gripper pointing down

    q = [q1 q2 q3 q4];
end
