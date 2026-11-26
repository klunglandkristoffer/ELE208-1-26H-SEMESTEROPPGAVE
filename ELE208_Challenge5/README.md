# ELE208 – Challenge 5

Modellering og visualisering av robot og arbeidsmiljø i Gazebo,
basert på gruppens arbeid i AutoCAD og MATLAB.

# Programvare

- Ubuntu 24.04.4 LTS
- ROS2 Jazzy
- Gazebo Harmonic 8.15.0
- Python 3.12
- Visual Studio Code
- VMware Workstation 17 Player

# Visualisering i Gazebo

Kjør følgende kommando i terminalen i Ubuntu,
fra prosjektets hovedmappe:

```bash
gz sim worlds/challenge5_final.sdf --render-engine ogre
```

Ogre 1 ble benyttet på grunn av stabilitetsproblemer i VM-en.

# Mappestruktur

- documentation: Arbeidslogg, referanser og KI-deklarasjon.
- models: Genererte robotmodeller i SDF-format.
- screenshots: Skjermbilder fra utviklingen i Gazebo.
- scripts: Python-program for generering av verdensfiler.
- urdf: Robotbeskrivelse i Xacro og URDF.
- worlds: Arbeidsmiljø og komplett robotmiljø i SDF-format.

# Kildefiler

- worlds/challenge5.sdf
- urdf/robot.urdf.xacro
- scripts/build_worlds.py

# Generering av verdensfiler

Med de eksisterende robotmodellene kan verdensfilene
genereres på nytt ved å kjøre:

```bash
python3 scripts/build_worlds.py
```

I Windows kan python benyttes i stedet for python3.

# Resultater og begrensninger

challenge5_final.sdf inneholder det fullførte statiske
robotmiljøet som ble visualisert i Gazebo.

challenge5_sim.sdf er en alternativ versjon for bevegelses-
simulering som ikke ble ferdig verifisert på grunn av
stabilitetsproblemer i den virtuelle maskinen.

Verdensfilene ble generert på nytt etter kodegjennomgang
i Windows, men ikke testet på nytt i Gazebo.

Se documentation for ytterligere informasjon om
utviklingsprosessen, faglige referanser og KI-bruk.