%% ELE208 - 4.2 Robot Model
clear; clc; close all;
enabled = true;

%% Robot dimensions, speeds, object size
% Limb lengths in [m]
L1 = 0.150; L2 = 1.060; L3 = 0.800; L4 = 0.263;
standoff = 0.1; % Distance between center of limbs

% Speeds
robot_speed    = 0.5;    % [rad/s]
conveyor_speed = 0.3;    % [m/s]
stepTime       = 0.05;   % [s] simulation time step (20 frames per second)

% Object size [m]
obj_side = 0.05;
obj_size = [obj_side, obj_side, obj_side];  % Cubic shape with obj_side sizes
hover_height = obj_size(3) * 1.5;           % Approach height above the pickup/drop point [m]
grip_offset = [0 0 obj_size(3)/2];          % Offset for grip. The gripper should grab the edge, not the center

%% Conveyor [m]
conv_dir   = [1 0 0];     % Belt moves along base +X (toward the robot)
conv_vel   = conveyor_speed * conv_dir;
belt_y     = -0.575;      % Belt centre line [m]
belt_w     = 0.40;        % Belt width [m]
belt_x     = [-2.5 2.5];  % Belt start and end [m]
pick_x_max = 1;           % Don't try to grab objects beyond this X [m]
x_spawn    = belt_x(1);   % Objects are spawned and detected at the beginning of the conveyor

%% Create joints using standard DH
J1 = Link([0 L1 0 pi/2 0], 'standard');
J2 = Link([0 standoff  L2 0 0], 'standard');
J3 = Link([0 -standoff L3 0 0], 'standard');
J4 = Link([0 standoff  L4 0 0], 'standard');
J2.offset = pi/2;

%% Create robot and define start position
robot = SerialLink([J1 J2 J3 J4], 'name', 'ELE208_4DOF');
q_zero = [0 0 0 0];
q_current = q_zero;

%% Positions [m]
s_table = [0.4, 0.4];                         % Table size
p_table = [0 -0.975 0];                       % Table position
p_table_obj = p_table + [0 0 obj_size(3)];    % Drop-off position for table
p_cam = [-2, -0.4, 1];                        % Camera position
T_base_cam = SE3(p_cam) * SE3.Ry(pi);         % Camera looks down
p_ready = [-0.5 belt_y hover_height];         % Waiting pose above the belt

%% 3D Environment
% Robot
figure;
robot.plot(q_zero, 'workspace', [-2.5 2.5 -2.5 2.5 0 2.5]);
title('ELE208 - 4DOF Robot');
hold on;

% Conveyor belt
fill3([belt_x(1) belt_x(2) belt_x(2) belt_x(1)], ...
      belt_y + belt_w/2*[-1 -1 1 1], ...
      0.002*ones(1,4), ...
      [0.4 0.4 0.4], 'EdgeColor', 'k');

% Drop-off table
fill3(p_table(1) + s_table(1)/2*[-1 1 1 -1], ...
      p_table(2) + s_table(2)/2*[-1 -1 1 1], ...
      0.002*ones(1,4), ...
      [0.2 0.7 0.2], 'EdgeColor', 'k');

%% Go to the ready pose
q_ready = solve_ik(robot, p_ready);
if isempty(q_ready), error('Ready pose unreachable.'); end
obj = [];
if enabled
    [q_current, obj] = move_to(robot, q_current, q_ready, robot_speed, stepTime, obj);
end

%% Automatic pick and place from a moving conveyor
while enabled
    % 1. Remove the last object and detect a new one
    if ~isempty(obj) && isgraphics(obj.h), delete(obj.h); end
    p_obj_cam = detect_object(obj_size, T_base_cam, belt_y, belt_w, x_spawn);

    % 2. Object -> Camera -> Robot base
    p_obj_base = T_base_cam.T * p_obj_cam;
    p_obj_base = p_obj_base(1:3)';
    obj = make_object(obj_size, p_obj_base, conv_vel);
    fprintf('Object detected at %s\n', mat2str(p_obj_base, 3));

    % 3. Plan the meeting point from the detected position and conveyor speed
    [q_hover, q_pick, p_pick, t_arrive, t_wait] = plan_meeting(robot, q_current, ...
    p_obj_base + grip_offset, conv_vel, hover_height, robot_speed, stepTime, pick_x_max);
    fprintf('  Pickup at %s. Arrive in %.2f s, then wait %.2f s.\n', mat2str(p_pick, 3), t_arrive, t_wait);

    % 4. Move to the hover point then wait for object to be under the gripper
    [q_current, obj] = move_to(robot, q_current, q_hover, robot_speed, stepTime, obj);
    for i = 1:round(t_wait / stepTime)
        obj = step_object(obj, [], stepTime);
        drawnow;
    end

    % 5. Descend while the object keeps moving, then grab
    [q_current, obj] = move_to(robot, q_current, q_pick, robot_speed, stepTime, obj);
    obj.mode = 'held';

    % 6. Lift, carry to the table, place it down
    up = [0 0 hover_height];
    q_lift = solve_ik(robot, p_pick + up);
    q_above_table = solve_ik(robot, p_table_obj + up);
    q_at_table = solve_ik(robot, p_table_obj);

    if isempty(q_lift) || isempty(q_above_table) || isempty(q_at_table)
        disp('Table waypoint unreachable.'); break; 
    end

    [q_current, obj] = move_to(robot, q_current, q_lift,      robot_speed, stepTime, obj);
    [q_current, obj] = move_to(robot, q_current, q_above_table, robot_speed, stepTime, obj);
    [q_current, obj] = move_to(robot, q_current, q_at_table,    robot_speed, stepTime, obj);
    obj.mode = 'static'; % Release

    % 7. Retreat and return to the ready pose
    [q_current, obj] = move_to(robot, q_current, q_above_table, robot_speed, stepTime, obj);
    [q_current, obj] = move_to(robot, q_current, q_ready,     robot_speed, stepTime, obj);
end

%% Local functions

function p_obj_cam = detect_object(obj_size, T_base_cam, belt_y, belt_w, x_spawn)
    % The object is placed on the belt in the BASE frame, then converted to the camera frame.
    y_range = belt_w/2 - obj_size(2)/2;          % keeps the whole cube on the belt
    y_base  = belt_y + y_range * (2*rand - 1);   % belt_y +/- y_range
    x_base  = x_spawn + 0.04*(rand - 0.5);       % small spread along the belt
    z_base  = obj_size(3)/2;                     % cube resting on the belt

    p_base = [x_base; y_base; z_base; 1];
    p_obj_cam = T_base_cam.T\p_base;      % base frame -> camera frame
end


function [q_hover, q_pick, p_pick, t_arrive, t_wait] = plan_meeting(robot, q_now, p0, v, hover_h, speed, stepTime, x_max)
    % Plan a meeting point from the detected position p0 and belt velocity v.
    % The object is at p_pick = p0 + v*t_meet at time t_meet.
    q_hover = []; q_pick = []; p_pick = [];
    t_arrive = NaN; t_wait = NaN;
    up = [0 0 hover_h];
    t_max = (x_max - p0(1)) / v(1);  % Object passes the pickup limit at this time

    for t_meet = 0:stepTime:t_max
        p_pick = p0 + v * t_meet;
        q_pick = solve_ik(robot, p_pick);
        if isempty(q_pick), continue; end

        % Descent time and hover point depend on each other, so iterate
        t_desc = 1.0;
        q_hover = [];
        for k = 1:6
            q_hover = solve_ik(robot, p_pick - v*t_desc + up);
            if isempty(q_hover), break; end
            t_new = move_time(q_hover, q_pick, speed, stepTime);
            if abs(t_new - t_desc) < 1e-9, break; end
            t_desc = t_new;
        end
        if isempty(q_hover), continue; end

        t_arrive = move_time(q_now, q_hover, speed, stepTime);
        t_start  = t_meet - t_desc;          % time at which the descent must begin
        if t_arrive <= t_start
            t_wait = t_start - t_arrive;     % time spent waiting above the belt
            return;
        end
    end
end


function q = solve_ik(robot, p)
    % Inverse kinematics, end-effector pointing straight down, elbow up.
    L1 = robot.d(1);               % Robot arm lengths
    L2 = robot.a(2);
    L3 = robot.a(3);
    L4 = robot.a(4);
    D  = sum(robot.d(2:4));        % Sideways offset of the tip

    r = hypot(p(1), p(2));
    if r < abs(D), q = []; return; end

    rho = sqrt(r^2 - D^2);
    q1  = atan2(p(2), p(1)) + atan2(D, rho);
    q1  = atan2(sin(q1), cos(q1));

    hw = p(3) - L1 + L4;            % Wrist height (end-effector points down)

    c3 = (rho^2 + hw^2 - L2^2 - L3^2) / (2*L2*L3);
    if abs(c3) > 1, q = []; return; end
    q3  = -acos(c3);
    th2 = atan2(hw, rho) - atan2(L3*sin(q3), L2 + L3*cos(q3));
    q2  = th2 - pi/2;
    q4  = -pi/2 - th2 - q3;

    q = [q1 q2 q3 q4];
end


function n = move_steps(q_from, q_to, speed, stepTime)
    n = max(2, ceil(max(abs(q_to - q_from)) / speed / stepTime));
end


function t = move_time(q_from, q_to, speed, stepTime)
    t = move_steps(q_from, q_to, speed, stepTime) * stepTime;
end


function [q_end, obj] = move_to(robot, q_from, q_to, speed, stepTime, obj)
    % Animate q_from -> q_to. The object is updated every frame based on its mode
    nSteps = move_steps(q_from, q_to, speed, stepTime);
    q_traj = jtraj(q_from, q_to, nSteps);
    for k = 1:nSteps
        robot.animate(q_traj(k, :));
        tip = robot.fkine(q_traj(k, :)).t';
        obj = step_object(obj, tip, stepTime);
        drawnow;
    end
    q_end = q_to;
end


function obj = make_object(sz, pos, vel)
    % 3D cube as a patch object
    s = sz/2;
    V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] .* s;
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    h = patch('Vertices', V + pos, 'Faces', F, 'FaceColor', [1 0.6 0], 'EdgeColor', 'k');
    obj = struct('h', h, 'V', V, 'pos', pos, 'vel', vel, 'mode', 'conveyor');
end


function obj = step_object(obj, tip, dt)
    % Mode that the cube is in
    if isempty(obj), return; end
    switch obj.mode
        case 'conveyor', obj.pos = obj.pos + obj.vel * dt;      % On conveyor, moves every frame
        case 'held',     obj.pos = tip - [0 0 max(obj.V(:,3))]; % Held by the robot, follows the tip
        case 'static'                                           % stays where it was released
    end
    set(obj.h, 'Vertices', obj.V + obj.pos);
end
