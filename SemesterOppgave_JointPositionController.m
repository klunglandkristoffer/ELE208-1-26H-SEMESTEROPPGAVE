%% ELE208 - 4iv Position controller for joint 1 (base rotation)
clear; clc; close all;

%% Motor and gearbox (DC motor with current amplifier, Corke ch. 9, PUMA joint 1)
% Same symbols as Corke's table: Km, Jm, Bm, G, tau_max
Km      = 0.228;      % Motor torque constant [N m A^-1]
Jm      = 200e-6;     % Motor inertia [kg m^2]
Bm      = 817e-6;     % Drive viscous friction [N m s rad^-1]
G       = 107.815;    % Gear ratio
tau_max = 0.900;      % Maximum motor torque [N m]
i_max   = tau_max/Km; % Current that gives tau_max [A]

%% Link inertia about joint 1, worst case: arm stretched horizontally [kg m^2]
% Aluminium tubes, 90 mm diameter, 3 mm wall -> 2.2 kg/m (see 4i)
L2 = 1.060; L3 = 0.800; L4 = 0.263; rho_l = 2.2;
m2 = rho_l*L2; m3 = rho_l*L3; m4 = rho_l*L4;
m_elbow = 1.5; m_wrist = 0.8; m_load = 0.5;   % Motors; gripper 0.4 + cube 0.1 (see 4i) [kg]
r_tip = L2 + L3;
J_link = m2*L2^2/3 + m3*(r_tip^3 - L2^3)/(3*L3) ...
       + (m4 + m_wrist + m_load)*r_tip^2 + m_elbow*L2^2;

%% Joint model seen from the link side
J_eq = G^2*Jm + J_link;          % Total inertia [kg m^2]
B_eq = G^2*Bm;                   % Total viscous friction [Nm s/rad]
% q'' = (G*Km*i - B_eq*q' - tau_d) / J_eq
P = ss([0 1; 0 -B_eq/J_eq], [0 0; G*Km/J_eq -1/J_eq], [1 0], 0, ...
       'InputName', {'i','tau_d'}, 'OutputName', 'q');
fprintf('J_link = %.2f kg m^2, J_eq = %.2f kg m^2\n', J_link, J_eq);
disp('Transfer function q(s)/i(s):'); tf(P(1,1))

%% PID controller, tuned for 10 rad/s crossover
C = pidtune(P(1,1), 'PIDF', 10);
C.InputName = 'e'; C.OutputName = 'i';
fprintf('Kp = %.1f A/rad, Ki = %.1f A/(rad s), Kd = %.1f A s/rad, Tf = %.4f s\n', ...
        C.Kp, C.Ki, C.Kd, C.Tf);
S  = sumblk('e = r - q');
CL = connect(P, C, S, {'r','tau_d'}, {'q','i'});

%% Test 1: step response
info = stepinfo(CL(1,1));
fprintf('Step: rise time %.3f s, settling time %.3f s, overshoot %.1f %%\n', ...
        info.RiseTime, info.SettlingTime, info.Overshoot);
[gm, pm] = margin(C*P(1,1));
fprintf('Gain margin %.1f dB, phase margin %.1f deg\n', 20*log10(gm), pm);
i_step = step(CL(2,1), 0:1e-4:2);
fprintf('Peak current for a 1 rad step: %.0f A -> use a smooth trajectory, not a raw step\n', ...
        max(abs(i_step)));

%% Test 2: follow the joint-1 move from 42ii1 (ready -> hover in 2.8 s)
dt = 1e-3; t = (0:dt:10)';
q_ready = deg2rad(-123.47); q_hover = deg2rad(-158.85); T_move = 2.8;
s = min(t/T_move, 1);
q_ref = q_ready + (q_hover - q_ready)*(6*s.^5 - 15*s.^4 + 10*s.^3);  % Quintic, like jtraj
tau_d = 20*(t >= 4);             % 20 Nm load disturbance at t = 4 s
% The model is linear, so simulate relative to the start angle (joint starts at rest in q_ready)
y = lsim(CL, [q_ref - q_ready, tau_d], t);
q = y(:,1) + q_ready; i_cmd = y(:,2);
fprintf('Max tracking error during move: %.3f deg\n', max(abs(rad2deg(q_ref(t<=T_move) - q(t<=T_move)))));
fprintf('Max error from disturbance:     %.3f deg\n', max(abs(rad2deg(q_ref(t>=4) - q(t>=4)))));
fprintf('Max current: %.2f A (limit %.2f A) -> max motor torque %.3f Nm (tau_max = %.3f Nm)\n', ...
        max(abs(i_cmd)), i_max, Km*max(abs(i_cmd)), tau_max);

%% Plots
figure('Color','w','Theme','light','Position',[100 100 700 350]);
step(CL(1,1)); grid on; title('Step response, joint 1 position');

figure('Color','w','Theme','light','Position',[100 100 700 600]);
subplot(3,1,1); plot(t, rad2deg(q_ref), '--', t, rad2deg(q)); grid on;
ylabel('q_1 [deg]'); legend('Reference','Actual'); title('Joint 1 tracking');
subplot(3,1,2); plot(t, rad2deg(q_ref - q)); grid on; ylabel('Error [deg]');
xline(4, ':', '20 Nm disturbance', 'LabelHorizontalAlignment', 'left');
subplot(3,1,3); plot(t, i_cmd); grid on; ylabel('Current [A]'); xlabel('Time [s]');
yline([-i_max i_max], 'r--');
