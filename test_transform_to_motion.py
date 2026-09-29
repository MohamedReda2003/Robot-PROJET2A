"""Test matériel : transforme une matrice cible en mouvement des six articulations.

Lancer avec l'interpréteur de l'environnement du projet :
    python test_transform_to_motion.py
"""

import numpy as np
from pyniryo import NiryoRobot

from Arm_kinematics import JOINT_LIMITS, mgd, mgi, pose_vec


ROBOT_IP = "10.10.3.156"

# Transformation outil/repère de base (mètres), exemple généré avec
# mgd([0.1, -0.3, 0.2, 0.0, 0.4, 0.0]).
# Remplacer cette matrice par la transformation cible souhaitée.
TARGET_TRANSFORM = np.array(
    [
        [0.9505637859221, -0.09983341664683, -0.2940438365519, 0.3487200184068],
        [0.09537450575679, 0.9950041652780, -0.02950279191918, 0.03498870869648],
        [0.2955202066613, 0.0, 0.9553364891256, 0.4089726606823],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=float,
)


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


def main():
    validate_transform(TARGET_TRANSFORM)
    target_pose = pose_vec(TARGET_TRANSFORM)

    print("Transformation cible outil/base :")
    print(TARGET_TRANSFORM)
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
        robot.calibrate_auto()
        current_joints = np.asarray(robot.get_joints(), dtype=float)
        if current_joints.shape != (6,) or not np.isfinite(current_joints).all():
            raise RuntimeError("PyNiryo n'a pas retourné six angles articulaires valides.")

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
        print("Test terminé.")
    finally:
        robot.close_connection()


if __name__ == "__main__":
    main()
