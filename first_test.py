from pyniryo import NiryoRobot
import time

# Adresse IP du Ned2
ROBOT_IP = "10.10.3.156"

# Connexion
robot = NiryoRobot(ROBOT_IP)

# Calibration
robot.calibrate_auto()

print("Robot connecté et calibré")

# Position de départ
robot.move_joints(
    -0.0,   # J1
    0.0,   # J2
    0.0,   # J3
    0.0,   # J4
    0.0,   # J5
    0.0    # J6
)
joints = robot.get_joints()

print("\n=== État actuel des articulations apres depart ===")

for i, angle in enumerate(joints):
    print(f"J{i + 1} : {angle:.2f} rad")
time.sleep(2)

# Petite rotation de J1
robot.move_joints(
    0.20,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
)

joints = robot.get_joints()

print("\n=== État actuel des articulations apres rotation de J1 ===")

for i, angle in enumerate(joints):
    print(f"J{i + 1} : {angle:.2f} rad")
time.sleep(2)

# Retour
robot.move_joints(
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
)

print("Test terminé")
joints = robot.get_joints()

print("\n=== État actuel des articulations apres retour ===")

for i, angle in enumerate(joints):
    print(f"J{i + 1} : {angle:.2f} rad")
robot.close_connection()