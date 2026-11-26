# KI-deklarasjon – ELE208 Challenge 5

# Generelt

Gruppens arbeid med robotdesign, AutoCAD, MATLAB og robotkinematikk danner grunnlaget for implementeringen i Gazebo.

ChatGPT-6.2 (OpenAI) ble brukt som hjelpemiddel til kodegenerering, supplerende beregninger, geometriske tilpasninger, veiledning og feilsøking.

# 1. Arbeidsmiljø – challenge5.sdf

SDF-strukturen og modellblokkene er generert med KI, basert på gruppens tegninger og matematiske modell.

Dette omfatter:
- Grunnmiljø, systemmoduler og gulv.
- Transportbånd (`conveyor_belt`) med fire støttebein.
- Base (`robot_mounting_plate`) og sokkel (`robot_pedestal`).
- Avlastningsbord (`dropoff_table`).
- Kamera med stativ (`camera_system`).
- Objekt på transportbåndet (`pick_object`).

Supplerende dimensjoner og koordinater ble bestemt med KI-assistanse for å få et mer realistisk arbeidsmiljø.

Valget av et felles referanseplan for MATLAB og Gazebo, der overflaten på transportbåndet ligger ved z = 0, var mitt eget. Modellene ble implementert, kontrollert og visualisert i Gazebo.

# 2. Robotmodell – robot.urdf.xacro

Robotens lengder, leddstruktur og DH-parametere bygger på gruppens kinematiske modell.

KI genererte Xacro-koden, inkludert:
- Parametere og argumenter.
- Makroene `cylinder_part`, `placeholder_inertia` og `robot_joint`.
- Robotens lenker og fire roterende ledd.
- Implementering av DH-transformasjoner i URDF.
- Geometrisk representasjon av vakuumgriper og TCP.

Masse, treghetsmomenter og leddbegrensninger er foreløpige verdier foreslått med KI-assistanse. De dynamiske parameterne er ikke verifisert mot en fysisk robotmodell.

Robotbeskrivelsen ble konvertert til URDF, validert og visualisert i Gazebo.

# 3. Python-program – build_worlds.py

Python-programmet er generert med KI for å sette sammen robotmodellen og arbeidsmiljøet.

Dette omfatter:
- Filbaner og programstruktur.
- Funksjonene `set_value`, `load_robot` og `save_world`.
- Generering av statisk og bevegelig robotmiljø.
- Forankring av robotens base.
- Oppsett av `JointPositionController` og `JointStatePublisher`.

Programmet ble gjennomgått, kommentert, syntakskontrollert og kjørt manuelt.

Den statiske modellen ble visualisert i Gazebo. Den alternative bevegelsessimuleringen ble ikke ferdig verifisert på grunn av stabilitetsproblemer i VM-en.

De automatisk genererte URDF- og SDF-filene bygger på den KI-genererte kildekoden.

# 4. Dokumentasjon

KI ble også benyttet til:
- Forklaring og gjennomgang av kode.
- Forslag til faglige kodekommentarer.
- Strukturering av arbeidslogg og KI-deklarasjoner.
- Innhenting av faglige referanser for etterkontroll.

Kodekommentarene er gjennomgått og tilpasset manuelt.

# 5. Eget arbeid

Det faglige grunnlaget for implementeringen er gruppens egne AutoCAD-tegninger, MATLAB-modell, robotdesign og kinematiske beregninger.

Den KI-genererte koden ble gjennomgått med utgangspunkt i gruppens kinematiske modell, geometriske dimensjoner, leddstruktur og valgte koordinatsystemer. Det ble gjort visuelle kontroller i Gazebo, men det ble ikke gjennomført en fullstendig numerisk verifikasjon mot MATLAB-modellen.

Praktisk oppsett, filhåndtering, implementering, testing, visualisering, feilsøking og dokumentering med skjermbilder ble gjennomført manuelt med KI-assistanse.

Ytterligere informasjon om utviklingsprosessen finnes i arbeidsloggen, utviklingsnotatene og kommentarene i kildefilene.