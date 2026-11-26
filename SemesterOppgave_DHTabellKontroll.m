%% 4.2.i.1
clear; clc;

% Robotens DH-tabell [theta_offset (grader), d (m), a (m), alpha (grader)]
DH = [ 0,  0.150, 0,     90;
      90,  0.100, 1.060,  0;
       0, -0.100, 0.800,  0;
       0,  0.100, 0.263,  0];
% Tre testkonfigurasjoner i grader
q_test = [  0,   0,   0,   0;
          -45, -40, -60,  20;
          -90, -60, -90, -30];
% Opprett robotmodell i Robotics Toolbox
for i = 1:4
    ledd(i) = Link([0, DH(i,2), DH(i,3), deg2rad(DH(i,4)), 0], 'standard');
    ledd(i).offset = deg2rad(DH(i,1));
end
robot = SerialLink(ledd);
% Test egen FK mot Robotics Toolbox
for k = 1:3
    q = deg2rad(q_test(k,:));
    T = eye(4);
    % Egen FK: multipliser de fire DH-matrisene
    for i = 1:4
        theta = q(i) + deg2rad(DH(i,1));
        d = DH(i,2);
        a = DH(i,3);
        alpha = deg2rad(DH(i,4));

        A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
             0,           sin(alpha),             cos(alpha),           d;
             0,           0,                      0,                    1];
        T = T*A;
    end

    % Samme leddvinkler i Robotics Toolbox
    T_toolbox = robot.fkine(q).T;

    %%ChatGPT bistått i linje 46 og 47 for sammenligning
    % Sammenlign posisjon og orientering
    pos_avvik = norm(T(1:3,4) - T_toolbox(1:3,4));
    rot_avvik = norm(T(1:3,1:3) - T_toolbox(1:3,1:3),'fro');

    %%ChatGPT generet kode for utskrift:
    fprintf('\nTest %d\n',k);
    disp('Egen FK:'); disp(T);
    disp('Robotics Toolbox:'); disp(T_toolbox);
    fprintf('Posisjonsavvik: %.3e m\n',pos_avvik);
    fprintf('Rotasjonsavvik: %.3e\n',rot_avvik);

    if pos_avvik <= 1e-6 && rot_avvik <= 1e-6
        disp('Test godkjent');
    else
        disp('Test feilet');
    end
end
