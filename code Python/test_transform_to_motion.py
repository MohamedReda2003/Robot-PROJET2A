"""Test matériel : saisit XYZ en cm, calcule la cible puis déplace le bras.

Lancer avec l'interpréteur de l'environnement du projet :
    python test_transform_to_motion.py

L'orientation de l'outil reste fixe selon TOOL_ORIENTATION (rad).
"""

import numpy as np
from pyniryo import NiryoRobot

from Arm_kinematics import JOINT_LIMITS, mgd, mgi, pose_vec


ROBOT_IP = "10.10.3.156"

# Orientation outil fixe (roll, pitch, yaw), en radians.
# Les coordonnées saisies sont exprimées par rapport à la base du robot.
TOOL_ORIENTATION = (0.0, -0.3, 0.1)


def build_target_transform(x_cm, y_cm, z_cm):
    """Construit la transformation cible à partir de XYZ en centimètres."""
    xyz_m = np.asarray([x_cm, y_cm, z_cm], dtype=float) / 100.0
    if not np.isfinite(xyz_m).all():
        raise ValueError("Les coordonnées doivent être des nombres finis.")

    roll, pitch, yaw = TOOL_ORIENTATION
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz_m
    return transform


def read_coordinate(name):
    while True:
        try:
            value = float(input(f"Entrez {name} en cm : ").replace(",", "."))
        except ValueError:
            print("Valeur invalide : entrez un nombre, par exemple 34.87.")
            continue
        if np.isfinite(value):
            return value
        print("La coordonnée doit être un nombre fini.")


def validate_transform(transform):
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError("La transformation cible doit être une matrice 4x4 finie.")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("La dernière ligne de la transformation doit être [0, 0, 0, 1].")

    rotation = transform[:3, :3]
    if (
        not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)
    ):
        raise ValueError("La partie rotation de la transformation n'est pas valide.")
    
def position_initiale(robot):
    """Lit les angles articulaires courants et renvoie (x, y, z) via le MGD."""
    q = robot.get_joints()
    x, y, z = mgd(q)[:3, 3]
    print(f"Position initiale : x={x:.3f} m, y={y:.3f} m, z={z:.3f} m")


def main():
    print("Saisissez la position cible de l'outil dans le repère de base du robot.")
    x_cm = read_coordinate("X")
    y_cm = read_coordinate("Y")
    z_cm = read_coordinate("Z")

    target_transform = build_target_transform(x_cm, y_cm, z_cm)
    validate_transform(target_transform)
    target_pose = pose_vec(target_transform)

    print("\nTransformation cible outil/base (translation en mètres) :")
    print(target_transform)
    print("\nPose cible [x, y, z, roll, pitch, yaw] (m, rad) :")
    print(np.array2string(target_pose, precision=6))

    if input("\nConnecter et calibrer le robot ? [o/N] ").strip().lower() not in {
        "o",
        "oui",
        "y",
        "yes",
    }:
        print("Test annulé, aucune connexion ni aucun mouvement effectué.")
        return

    robot = NiryoRobot(ROBOT_IP)
    try:
        position_initiale(robot)
        robot.calibrate_auto()
        current_joints = np.asarray(robot.get_joints(), dtype=float)
        if current_joints.shape != (6,) or not np.isfinite(current_joints).all():
            raise RuntimeError("PyNiryo n'a pas retourné six angles articulaires valides.")

        MAX_REACH = 0.539  # m, = 0.221326+0.0325+0.235+0.00925+0.0412
        Z_MIN, Z_MAX = 0.02, 0.75  # à ajuster selon ta table/support

        def check_reachable(x_cm, y_cm, z_cm):
            r = np.hypot(x_cm, y_cm)
            if r > MAX_REACH * 100:
                raise ValueError(f"Hors de portée : distance radiale {r:.1f} cm > {MAX_REACH*100:.0f} cm max.")
            if not (Z_MIN * 100 <= z_cm <= Z_MAX * 100):
                raise ValueError(f"Hors de portée : Z={z_cm:.1f} cm hors de [{Z_MIN*100:.0f}, {Z_MAX*100:.0f}] cm.")

        check_reachable(x_cm, y_cm, z_cm)
        target_joints = mgi(target_pose, q0=current_joints)
        if any(
            angle < lower - 1e-6 or angle > upper + 1e-6
            for angle, (lower, upper) in zip(target_joints, JOINT_LIMITS)
        ):
            raise RuntimeError("La solution MGI dépasse les limites articulaires configurées.")

        print("\nArticulations actuelles (rad) :")
        print(np.array2string(current_joints, precision=4))
        print("Articulations calculées par le MGI (rad) :")
        print(np.array2string(target_joints, precision=4))
        print("Position demandée (cm) :", (x_cm, y_cm, z_cm))

        if input("\nEnvoyer ce mouvement au robot ? [o/N] ").strip().lower() not in {
            "o",
            "oui",
            "y",
            "yes",
        }:
            print("Mouvement annulé.")
            return

        robot.move_joints(*target_joints.tolist())
        actual_joints = np.asarray(robot.get_joints(), dtype=float)
        actual_pose = pose_vec(mgd(actual_joints))

        print("\nArticulations après le mouvement (rad) :")
        print(np.array2string(actual_joints, precision=4))
        print("Pose estimée après le mouvement [x, y, z, roll, pitch, yaw] (m, rad) :")
        print(np.array2string(actual_pose, precision=6))
        print("Position estimée après le mouvement (cm) :")
        print(np.array2string(actual_pose[:3] * 100.0, precision=2))
        pos_err = np.linalg.norm(actual_pose[:3] - target_pose[:3]) * 100
        print(f"Écart position demandée/obtenue : {pos_err:.2f} cm")
        if pos_err > 1.0:
            print("ATTENTION : écart > 1 cm — vérifier calibration ou DH.")
        print("Test terminé.")
    finally:
        robot.close_connection()


if __name__ == "__main__":
    main()
