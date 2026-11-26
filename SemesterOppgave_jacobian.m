%% 4.2.i.2
clear; clc;

% DH-tabell [theta_offset (grader), d (m), a (m), alpha (grader)]
DH = [ 0,  0.150, 0,     90;
      90,  0.100, 1.060,  0;
       0, -0.100, 0.800,  0;
       0,  0.100, 0.263,  0];

% Leddvinkler fra analytisk IK ved beltets senter, ca. [0 -0.575 0.025] i BASE.

q = deg2rad([-79.984590, -28.086326, -147.096002, -4.817672]);   % Summen av q2, q3 og q4 er -180 grader (gripeaksen peker ned).
v_belte = [0.30; 0; 0];                                          % Ønsket TCP-hastighet [m/s]
qdot_max = [1.30; 0.60; 1.30; 0.74];                             % Antatte leddhastigheter [rad/s]

% Samme FK-beregning som i "SemesterOppgave_42ii", men lagrer leddenes akser og posisjoner.
T = eye(4);
for i = 1:4
    z_axes(:,i) = T(1:3,3);
    origins(:,i) = T(1:3,4);
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

% Geometrisk Jacobian: lineær hastighet og vinkelhastighet.
p = T(1:3,4);
for i = 1:4
    J(1:3,i) = cross(z_axes(:,i),p-origins(:,i));
    J(4:6,i) = z_axes(:,i);
end

% Oppgave-Jacobian: tre hastigheter og konstant gripevinkel.
A = [J(1:3,:); 0 1 1 1];
if rcond(A) < 1e-7
    error('Jacobian er singulær eller nær singulær.');
end
qdot = A \ [v_belte; 0];

% Kontroll: beregn TCP-hastighet fra leddhastighetene.
v = J(1:3,:)*qdot;

% ChatGPT generert utskrift:
fprintf('Differensiell kinematikk\n');
disp('TCP-posisjon i BASE [m]:'); disp(p');
disp('Lokal X-akse:'); disp(T(1:3,1)');
disp('Geometrisk Jacobian:'); disp(J);
disp('Nødvendige leddhastigheter [rad/s]:'); disp(qdot');
disp('Beregnet TCP-hastighet [m/s]:'); disp(v');
fprintf('Hastighetsavvik: %.3e m/s\n',norm(v-v_belte));
if all(abs(qdot) <= qdot_max+1e-9) && norm(v-v_belte) <= 1e-7 && ...
        abs(sum(qdot(2:4))) <= 1e-7 && norm(T(1:3,1)-[0;0;-1]) <= 1e-6
    disp('Test godkjent');
else
    disp('Test feilet');
end
