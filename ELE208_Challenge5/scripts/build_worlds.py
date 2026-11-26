# ELE208 - Challenge 5
# Generering av statisk og bevegelig robotmiljø

# AI/KI-deklarasjon:
# Python-programmet med tilhørende funksjoner er generert
# med ChatGPT-6.2 for å sette sammen roboten og arbeidsmiljøet i en SDF-fil.
#
# Det er også laget en alternativ versjon med posisjonsstyring
# av robotens fire ledd. Denne ble ikke ferdig testet på grunn
# av problemer med stabiliteten til VM-en.
#
# Se kommentarer underveis for forklaring av funksjonene og valgene som er gjort.

# Biblioteker for håndtering av filbaner, XML-filer og kopiering av modeller.

from pathlib import Path
import xml.etree.ElementTree as ET
import copy


# Finner prosjektets hovedmappe ut fra plasseringen til denne Python-filen.

ROOT = Path(__file__).resolve().parent.parent


# Henter arbeidsmiljøet og de to robotmodellene som skal brukes til generering av SDF-filene.

WORLD_FILE = ROOT / "worlds/challenge5.sdf"
STATIC_ROBOT = ROOT / "models/robot_display.sdf"
MOVING_ROBOT = ROOT / "models/robot_zero.sdf"


# Navn på robotens fire roterende ledd fra robotmodellen.

JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4"
]


# Endrer verdien til et XML-element. Dersom elementet ikke finnes, opprettes det.

def set_value(parent, tag, value):
    element = parent.find(tag)

    if element is None:
        element = ET.SubElement(parent, tag)

    element.text = str(value)
    return element


# Leser SDF-filen og henter ut robotmodellen.
# Gir en feilmelding dersom modellen ikke finnes.

def load_robot(path):
    model = ET.parse(path).getroot().find("model")

    if model is None:
        raise RuntimeError(f"Fant ingen robotmodell i {path}")

    return model


# Setter sammen robotmodellen og arbeidsmiljøet og lagrer resultatet som en ny SDF-fil.

def save_world(robot, filename, world_name):
    parser = ET.XMLParser(                              # Leser XML-filen og beholder kommentarer inne i XML-strukturen.
        target=ET.TreeBuilder(insert_comments=True)
    )

    tree = ET.parse(WORLD_FILE, parser=parser)
    world = tree.getroot().find("world")

    world.set("name", world_name)

    # Fjern eventuell tidligere robotintegrasjon
    for element in list(world):
        if element.tag == "model":
            if element.get("name") == "challenge5_robot":
                world.remove(element)

        elif element.tag == "include":
            if element.findtext("name") == "challenge5_robot":
                world.remove(element)


    # Legger en kopi av robotmodellen inn i arbeidsmiljøet.
    
    world.append(copy.deepcopy(robot))

    
    # Formaterer XML-strukturen før filen lagres.
    
    ET.indent(tree, space="    ")

    output = ROOT / "worlds" / filename

    tree.write(
        output,
        encoding="utf-8",
        xml_declaration=True
    )

    print(f"Opprettet: {output}")


# Henter roboten med faste ledd i ønsket visningsstilling.
# Roboten plasseres i origo på basen i arbeidsmiljøet.

robot = load_robot(STATIC_ROBOT)

robot.set("name", "challenge5_robot")

set_value(robot, "static", "true")
set_value(robot, "pose", "0 0 0 0 0 0")

save_world(                                     # Lagrer det komplette stillestående robotmiljøet.
    robot,
    "challenge5_final.sdf",
    "challenge5_final"
)


# Alternativ robotmodell med fire roterende ledd.
# Brukes til forsøk på bevegelse av roboten i Gazebo.

robot = load_robot(MOVING_ROBOT)

robot.set("name", "challenge5_robot")

set_value(robot, "static", "false")
set_value(robot, "pose", "0 0 0 0 0 0")
set_value(robot, "self_collide", "false")           # Deaktiverer kollisjoner mellom robotens egne lenker.


# Kontroller at alle fire ledd finnes

joint_names = {
    joint.get("name")
    for joint in robot.findall("joint")
    if joint.get("type") == "revolute"
}

for name in JOINTS:
    if name not in joint_names:
        raise RuntimeError(f"Mangler revolute joint: {name}")


# Deaktiverer gravitasjon på robotens lenker siden masse og treghetsmomenter ikke er fysisk dimensjonert.

for link in robot.findall("link"):
    set_value(link, "gravity", "false")


# Fest robotbasen til verdenskoordinatsystemet

if robot.find("link[@name='base_link']") is None:
    raise RuntimeError("Fant ikke base_link")

anchor = ET.SubElement(
    robot,
    "joint",
    name="base_anchor",             # Oppretter et fast ledd mellom base_link og verden slik at robotens base ikke beveger seg.
    type="fixed"
)

set_value(anchor, "parent", "world")
set_value(anchor, "child", "base_link")


# Oppsett av posisjonsstyring med Gazebos innebygde
# JointPositionController, foreslått og implementert med KI.
# Oppretter en kontroller for hvert av de fire leddene.

for name in JOINTS:

    controller = ET.SubElement(
        robot,
        "plugin",
        filename="gz-sim-joint-position-controller-system",
        name="gz::sim::systems::JointPositionController"
    )

    set_value(controller, "joint_name", name)
    set_value(controller, "initial_position", "0")
    set_value(controller, "use_velocity_commands", "true")  ## Posisjonsstyring ved hjelp av hastighetskommandoer, uten bruk av PID-regulator.

    # Foreløpig maksimal leddhastighet på 0.6 rad/s.
    set_value(controller, "cmd_max", "0.6")
    set_value(controller, "cmd_min", "-0.6")


    # Hvert ledd får en egen topic for mottak av posisjonskommandoer.
    
    set_value(
        controller,
        "topic",
        f"/challenge5/{name}/cmd_pos"
    )


# Oppretter en plugin som skal publisere robotens leddtilstander slik at leddposisjonene kan leses under simulering.

publisher = ET.SubElement(
    robot,
    "plugin",
    filename="gz-sim-joint-state-publisher-system",
    name="gz::sim::systems::JointStatePublisher"
)

for name in JOINTS:
    ET.SubElement(publisher, "joint_name").text = name


# Lagrer den alternative modellen for bevegelsessimulering.
# Denne versjonen ble ikke ferdig testet grunnet tekniske begrensninger relatert til VM'en.

save_world(
    robot,
    "challenge5_sim.sdf",
    "challenge5_sim"
)

print("Begge Gazebo-miljoene er generert.")