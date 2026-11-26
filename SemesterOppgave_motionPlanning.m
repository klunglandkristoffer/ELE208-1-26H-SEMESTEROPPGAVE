%% 42i4
% Kartesisk planlegging av en pick-and-place-sekvens.
% KI har bistatt med organisering, visualisering- og utskrifts-linjer.
clear; clc; close all;

%% Koordinater til "verden"
robot_base = [0, 0.200, 0];
belte_senter = [0,-0.375,0];
belte_x = [-2.5,2.5];
belte_bredde = 0.400;
bord_storrelse = [0.500,0.350];
dropoff = [0,-0.825,0.025];
v_belte = [0.30,0,0];

%% planlagte posisjoner og tider:

%En oppgitt start posisjon for objektet: i praksis vil det være kamera som
%gir denne infoen.
objekt_start = belte_senter + [-0.60,0,0.025];

% Start posisijonen til end-effektoren:
robot_start = belte_senter + [-0.70,0,0.300];

% tiden hver fase skal ta:
tid = [2.00,0.35,1.55,2.00,1.30,1.30,2.50];

%% De 7 fasene navngitt:
faser = {'Innkjøring','Tracking','Løft','Transport','Nedsetting','Klarering','Retur'};

% Beregning av objektets fremtidige posisjon. Dette er fra når kamera ville tatt
% bilde og sendt over til robot (*tiden og meterene pr sekund)
pickup = objekt_start + tid(1)*v_belte;
tracking = pickup + tid(2)*v_belte;
loft = tracking + [0.120,0,0.250];
over_bord = [dropoff(1:2),belte_senter(3)+0.275];

% Posisjon og hastighet ved hver faseovergang.
punkter = [robot_start; pickup; tracking; loft; over_bord; dropoff; over_bord; robot_start];
hastigheter = [0 0 0; v_belte; v_belte; zeros(5,3)];
% Start- og sluttakselerasjon er null i alle faser.

% bistand fra ChatGPT for "feilsøking", mindre skrivefeil som gjorde at vi
% ikke fikk det til. (linje 45 til 50)
%% Beregn kartesisk bane med femtegradspolynom
P = []; V = []; t = []; fase = [];
for i = 1:7
    T = tid(i);
    tau = linspace(0,T,round(T/0.01)+1)';
    p0 = punkter(i,:); p1 = punkter(i+1,:);
    v0 = hastigheter(i,:); v1 = hastigheter(i+1,:);

    % Koeffisienter for normalisert tid s = tau/T.
    c0 = p0;
    c1 = T*v0;
    % c2 = 0 fordi startakselerasjonen er null.
    C = [1 1 1; 3 4 5; 6 12 20] \ [p1-c0-c1; T*v1-c1; 0 0 0];
    c3 = C(1,:); c4 = C(2,:); c5 = C(3,:);

   %variabler/polynomer:
    s = tau/T;
    P = [P; c0+s*c1+s.^3*c3+s.^4*c4+s.^5*c5];
    V = [V; (c1+3*s.^2*c3+4*s.^3*c4+5*s.^4*c5)/T];
    t = [t; sum(tid(1:i-1))+tau];
    fase = [fase; i*ones(numel(tau),1)];
end

%%ChatGPT:
%% Visualisering av planlagt TCP-bane (KI-assistert)
figure('Name','Motion planning - TCP-bane','Color','w'); hold on;
xb = belte_x + belte_senter(1);
yb = belte_senter(2)+[-1 1]*belte_bredde/2;
patch([xb(1) xb(2) xb(2) xb(1)],[yb(1) yb(1) yb(2) yb(2)], ...
    belte_senter(3)*ones(1,4),[.3 .6 .9],'FaceAlpha',.2,'DisplayName','Belte');
xbord = dropoff(1)+[-1 1]*bord_storrelse(1)/2;
ybord = dropoff(2)+[-1 1]*bord_storrelse(2)/2;
patch([xbord(1) xbord(2) xbord(2) xbord(1)],[ybord(1) ybord(1) ybord(2) ybord(2)], ...
    (dropoff(3)-0.025)*ones(1,4),[.9 .6 .2],'FaceAlpha',.3,'DisplayName','Bord');
farger = lines(7);
for i = 1:7
    ids = find(fase==i);
    if i>1, ids = [ids(1)-1;ids]; end
    plot3(P(ids,1),P(ids,2),P(ids,3),'LineWidth',2, ...
        'Color',farger(i,:),'DisplayName',faser{i});
end
plot3(robot_base(1),robot_base(2),robot_base(3),'ko','MarkerFaceColor','k','DisplayName','Robotbase');
plot3(pickup(1),pickup(2),pickup(3),'mp','MarkerFaceColor','m','DisplayName','Pickup');
plot3(dropoff(1),dropoff(2),dropoff(3),'rs','MarkerFaceColor','r','DisplayName','Drop-off');
axis equal; grid on; view(35,25);
xlabel('X WORLD [m]'); ylabel('Y WORLD [m]'); zlabel('Z WORLD [m]');
legend('Location','eastoutside'); title('Planlagt TCP-bane');

%%ChatGPT:
%% Visualisering av planlagt TCP-hastighet (KI-assistert)
figure('Name','Motion planning - TCP-hastighet','Color','w');
plot(t,sqrt(sum(V.^2,2)),'LineWidth',1.5); hold on;
yline(norm(v_belte),'--','Transportbaand 0.30 m/s');
xlabel('Tid [s]'); ylabel('TCP-hastighet [m/s]'); grid on;
title('Planlagt TCP-hastighet gjennom arbeidssekvensen');
