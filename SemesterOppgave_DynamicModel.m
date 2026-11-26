%% ELE208 - 4i Dynamic model of the robot
clear; clc; close all;

%% Geometry [m] (same as a02_RobotModel_REV5)
L1 = 0.150; L2 = 1.060; L3 = 0.800; L4 = 0.263;
standoff = 0.1;

%% Material and component masses
% Links are aluminium tubes, 90 mm outer diameter, 3 mm wall
rho_al = 2700;                          % [kg/m^3]
ro = 0.045; ri = ro - 0.003;            % Outer/inner radius [m]
rho_l = rho_al*pi*(ro^2 - ri^2);        % Mass per metre, approx 2.2 kg/m
m_shoulder = 2.5;   % Joint 2 motor + gearbox, sits at the top of link 1 [kg]
m_elbow    = 1.5;   % Joint 3 motor + gearbox, end of link 2 [kg]
m_wrist    = 0.8;   % Joint 4 motor + gearbox, end of link 3 [kg]
m_gripper  = 0.4;   % Vacuum gripper incl. suction cup, end of link 4 [kg]
m_cube     = 0.1;   % Payload: 50 mm cube [kg]

%% Mass, centre of mass and inertia for each link (in its own DH frame)
% Standard DH: frame i sits at the far end of link i, the link lies along -x_i
% (along -y_1 for link 1, which is vertical). Point masses sit at the frame origin.
[m1, r1, I1] = tube_link(rho_l, ro, ri, L1, 2, m_shoulder);   % Link 1 along y_1
[m2, r2, I2] = tube_link(rho_l, ro, ri, L2, 1, m_elbow);      % Link 2 along x_2
[m3, r3, I3] = tube_link(rho_l, ro, ri, L3, 1, m_wrist);      % Link 3 along x_3
[m4, r4, I4] = tube_link(rho_l, ro, ri, L4, 1, m_gripper + m_cube); % Link 4 along x_4, incl. cube

%% Motor and gearbox parameters (PUMA 560 values, Corke ch. 9)
% Joint 2 uses the joint-1 gearbox (G = 107.8) instead of the PUMA's 53.7:
% our arm is longer, and 53.7 gives only 48 Nm while holding the arm needs up to 88 Nm.
% Same symbols as Corke's table: Km, Jm, Bm, tau_C+/tau_C-, G, tau_max, qd_max
Km      = 0.228;                               % Motor torque constant [N m A^-1]
Jm      = [200e-6  200e-6  200e-6  33e-6];     % Motor inertia [kg m^2]
Bm      = [817e-6  1.48e-3 1.38e-3 71.2e-6];   % Drive viscous friction [N m s rad^-1]
tau_C   = [0.126   -0.071;                     % Drive Coulomb friction [tau_C+ tau_C-] [N m]
           0.0712  -0.0733;
           0.105   -0.0989;
           11.3e-3 -16.9e-3];
G       = [107.815 107.815 76.0364 71.923];    % Gear ratio
tau_max = 0.900;                               % Maximum motor torque [N m]
qd_max  = 165;                                 % Maximum motor speed [rad s^-1]

%% Build the robot with dynamics
dh = [0 L1 0 pi/2; 0 standoff L2 0; 0 -standoff L3 0; 0 standoff L4 0];
m = [m1 m2 m3 m4]; r = [r1; r2; r3; r4]; I = cat(3, I1, I2, I3, I4);
for k = 1:4
    L(k) = Link(dh(k,:), 'standard'); %#ok<SAGROW>
    L(k).m = m(k); L(k).r = r(k,:); L(k).I = I(:,:,k);
    L(k).Jm = Jm(k); L(k).B = Bm(k); L(k).Tc = tau_C(k,:); L(k).G = G(k);  % Toolbox names
end
L(2).offset = pi/2;
robot = SerialLink(L, 'name', 'ELE208_4DOF');
robot.gravity = [0 0 9.81];

%% Print the parameter table
fprintf('Tube mass per metre: %.2f kg/m\n\n', rho_l);
fprintf('Link   m [kg]   r_c [m]                 Ixx       Iyy       Izz [kg m^2]\n');
for k = 1:4
    fprintf('%d    %6.2f   [%6.3f %6.3f %6.3f]   %8.4f  %8.4f  %8.4f\n', ...
            k, m(k), r(k,:), I(1,1,k), I(2,2,k), I(3,3,k));
end
fprintf('Total moving mass incl. cube: %.2f kg\n\n', sum(m));

%% Use the model in the task poses from 42ii1
q_names = {'Ready', 'Hover', 'Pickup', 'Stretched'};
Q = deg2rad([-123.5 -27.3 -133.0 -19.6;     % Ready
             -158.9 -59.2  -53.2 -67.6;     % Hover
             -152.6 -44.6  -93.1 -42.3;     % Pickup
                0   -90      0   -90  ]);   % Worst case: arm horizontal, gripper down
tau_joint_max = tau_max*G;                  % Max torque at the joint, after the gear [N m]
qd_joint_max  = qd_max./G;                  % Max joint speed [rad/s]
fprintf('Pose        M11 [kg m^2]   Gravity torque [Nm]: q1     q2     q3     q4\n');
for k = 1:size(Q,1)
    M = robot.inertia(Q(k,:));
    tg = robot.gravload(Q(k,:));
    fprintf('%-10s  %8.2f                  %7.1f %7.1f %7.1f %7.1f\n', ...
            q_names{k}, M(1,1), tg);
end
fprintf('Max joint torque  tau_max*G: %s Nm\n', mat2str(tau_joint_max, 3));
fprintf('Max joint speed   qd_max/G:  %s rad/s\n', mat2str(qd_joint_max, 3));
fprintf('Reflected motor inertia G^2*Jm: %s kg m^2\n', mat2str(G.^2.*Jm, 3));

%% Plot: how inertia and gravity torque change during the ready -> hover move
qt = jtraj(Q(1,:), Q(2,:), 100);
M11 = zeros(100,1); M22 = M11; tg = zeros(100,4);
for k = 1:100
    M = robot.inertia(qt(k,:));
    M11(k) = M(1,1); M22(k) = M(2,2);
    tg(k,:) = robot.gravload(qt(k,:));
end
s = linspace(0, 1, 100);
figure('Color','w','Theme','light','Position',[100 100 700 500]);
subplot(2,1,1); plot(s, M11, s, M22); grid on;
ylabel('Inertia [kg m^2]'); legend('M_{11} (joint 1)','M_{22} (joint 2)');
title('Ready \rightarrow hover: configuration-dependent inertia');
subplot(2,1,2); plot(s, tg(:,2:4)); grid on;
ylabel('Gravity torque [Nm]'); xlabel('Normalised path [-]');
legend('Joint 2','Joint 3','Joint 4');

figure('Color','w','Theme','light');
robot.plot(Q(4,:), 'workspace', [-0.5 2.5 -1 1 -0.5 1.5]);
title('Worst-case pose: arm horizontal');

%% Local functions
function [m, rc, Ic] = tube_link(rho_l, ro, ri, Llen, ax, m_pt)
    % Tube of length Llen along -axis 'ax' from the frame origin, plus a point mass
    % m_pt at the origin. Returns total mass, centre of mass and inertia about the CoM.
    m_t = rho_l*Llen;
    I_ax = m_t/2*(ro^2 + ri^2);                       % About the tube's own axis
    I_tr = m_t/12*(3*(ro^2 + ri^2) + Llen^2);         % Transverse, about tube centre
    I_t = I_tr*ones(1,3); I_t(ax) = I_ax;
    r_t = zeros(1,3); r_t(ax) = -Llen/2;              % Tube centre
    m  = m_t + m_pt;
    rc = m_t*r_t/m;                                   % Point mass is at the origin
    % Parallel axis theorem to move both parts to the common CoM
    d_t = r_t - rc; d_p = -rc;
    Ic = diag(I_t) + m_t*(dot(d_t,d_t)*eye(3) - d_t'*d_t) ...
                   + m_pt*(dot(d_p,d_p)*eye(3) - d_p'*d_p);
end
