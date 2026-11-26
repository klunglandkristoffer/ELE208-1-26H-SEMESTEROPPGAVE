# Arbeidslogg – ELE208 Challenge 5

# 1. Oppstart og grunnmiljø
- [x] Kontroll av Ubuntu 24.04.4 LTS, ROS2 Jazzy og Gazebo Harmonic 8.15.0.
- [x] Opprettelse av prosjektstruktur i VS Code.
- [x] Opprettelse av grunnmiljø i SDF med gulv, lys og systemmoduler.
- [x] Validering og visualisering i Gazebo.

# 2. Modellering av arbeidsmiljø
- [x] Felles referanseplan ved z = 0 for MATLAB og Gazebo.
- [x] Transportbånd med dimensjoner fra AutoCAD.
- [x] Støttebein til transportbåndet.
- [x] Robotfundament med base og sokkel.
- [x] Avlastningsbord basert på MATLAB-modellen.
- [x] Kamera med stativ.
- [x] Objekt på transportbåndet.
- [x] Visuell kontroll av arbeidsmiljøet i Gazebo.

# 3. Modellering av robot
- [x] Opprettelse av Xacro-fil.
- [x] Implementering av gruppens DH-parametere.
- [x] Modellering av fire roterende ledd og tilhørende lenker.
- [x] Modellering av vakuumgriper og TCP.
- [x] Konvertering fra Xacro til URDF.
- [x] Validering med check_urdf.
- [x] Konvertering fra URDF til SDF.
- [x] Visualisering av robot i Gazebo.

# 4. Integrering av robot og arbeidsmiljø
- [x] Valg av alternativ visningsstilling for roboten.
- [x] Opprettelse av build_worlds.py.
- [x] Generering av komplett robotmiljø.
- [x] Visualisering og dokumentasjon med skjermbilder.

# 5. Forsøk på bevegelsessimulering
- [x] Opprettelse av alternativ modell med roterende ledd.
- [x] Oppsett av JointPositionController.
- [x] Oppsett av JointStatePublisher.
- [x] Feilsøking av stabilitetsproblemer i VM.
- [ ] Verifisering av robotens bevegelse i Gazebo.

Bevegelsessimuleringen ble ikke ferdig testet på grunn
av gjentatte stabilitetsproblemer i den virtuelle maskinen.
Bytte fra Ogre 2 til Ogre 1, deaktivering av 3D-akselerasjon
og andre feilsøkingstiltak ble forsøkt.

# 6. Gjennomgang og dokumentasjon
- [x] Sikkerhetskopiering av prosjektet.
- [x] Overføring av prosjektfiler til Windows.
- [x] Gjennomgang og kommentering av SDF-kode.
- [x] Gjennomgang og kommentering av Xacro-kode.
- [x] Gjennomgang og kommentering av Python-kode.
- [x] KI-deklarasjoner i kildefilene.
- [x] Syntakskontroll av Python-programmet.
- [x] Regenerering av begge verdensfilene i Windows.
- [ ] Ny validering i Gazebo etter kodegjennomgang.
- [ ] Kontroll av kameraets koordinater mot endelig MATLAB-modell.
- [ ] Faglige referanser.
- [ ] KI-vedlegg til rapporten.

# KI-bruk
ChatGPT-6.2 ble benyttet til kodegenerering, veiledning,
koordinatberegninger, geometriske tilpasninger,
feilsøking og gjennomgang av kode og dokumentasjon.

Implementasjonen tar utgangspunkt i gruppens arbeid
i AutoCAD og MATLAB.

Mer detaljert beskrivelse av KI-bruken finnes i
kildefilenes KI-deklarasjoner og rapportens KI-vedlegg.