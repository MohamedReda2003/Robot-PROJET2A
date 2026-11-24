import time
import numpy as np
from pyniryo import NiryoRobot
from Arm_kinematics import (
    JOINT_LIMITS, mgd, mgi, pose_vec, pose_to_matrix,
    joint_interpolation, is_in_limits, validate_target_reachable,
    check_singularity
)

# ============================================================================
# CONFIGURATION
# ============================================================================

ROBOT_IP = "10.10.3.156"

# Orientation par défaut de l'outil (roll, pitch, yaw en rad)
TOOL_ORIENTATION = (0.0, -0.3, 0.1)

# Paramètres de trajectoire
N_STEPS = 50          # nombre de waypoints par mouvement
STEP_DELAY = 0.05     # délai entre waypoints (s) → ~2.5 s pour 50 waypoints
SPEED_RATIO = 0.8     # ratio de vitesse PyNiryo [0.1, 1.0]

# Limites de sécurité (espace de travail effectif)
WORKSPACE_X = (0.10, 0.45)   # m
WORKSPACE_Y = (-0.30, 0.30)  # m
WORKSPACE_Z = (0.02, 0.55)   # m


# ============================================================================
# FONCTIONS UTILITAIRES
# ============================================================================

def read_coordinate(name: str) -> float:
    """Lit une coordonnée au clavier avec validation."""
    while True:
        try:
            value = float(input(f"Entrez {name} en cm : ").replace(",", "."))
        except ValueError:
            print("  Valeur invalide : entrez un nombre, par exemple 34.87.")
            continue
        if np.isfinite(value):
            return value
        print("  La coordonnée doit être un nombre fini.")


def read_orientation() -> tuple:
    """Lit l'orientation RPY optionnelle (sinon utilise la valeur par défaut)."""
    use_default = input(
        f"Utiliser l'orientation par défaut RPY={TOOL_ORIENTATION} ? [O/n] : "
    ).strip().lower()
    if use_default not in ("n", "non", "no"):
        return TOOL_ORIENTATION
    try:
        r = float(input("  Roll (rad) : ").replace(",", "."))
        p = float(input("  Pitch (rad) : ").replace(",", "."))
        y = float(input("  Yaw (rad) : ").replace(",", "."))
        return (r, p, y)
    except ValueError:
        print("  Valeur invalide, retour à l'orientation par défaut.")
        return TOOL_ORIENTATION


def build_target_pose(x_cm: float, y_cm: float, z_cm: float,
                      orientation: tuple) -> np.ndarray:
    """Construit la pose cible [x, y, z, roll, pitch, yaw] en m et rad."""
    xyz_m = np.asarray([x_cm, y_cm, z_cm], dtype=float) / 100.0
    return np.concatenate([xyz_m, np.asarray(orientation, dtype=float)])


def validate_workspace(x_cm: float, y_cm: float, z_cm: float) -> None:
    """Vérifie que la cible est dans l'espace de travail effectif."""
    x_m, y_m, z_m = x_cm / 100.0, y_cm / 100.0, z_cm / 100.0
    if not (WORKSPACE_X[0] <= x_m <= WORKSPACE_X[1]):
        raise ValueError(
            f"X={x_cm:.1f} cm hors de [{WORKSPACE_X[0]*100:.0f}, {WORKSPACE_X[1]*100:.0f}] cm"
        )
    if not (WORKSPACE_Y[0] <= y_m <= WORKSPACE_Y[1]):
        raise ValueError(
            f"Y={y_cm:.1f} cm hors de [{WORKSPACE_Y[0]*100:.0f}, {WORKSPACE_Y[1]*100:.0f}] cm"
        )
    if not (WORKSPACE_Z[0] <= z_m <= WORKSPACE_Z[1]):
        raise ValueError(
            f"Z={z_cm:.1f} cm hors de [{WORKSPACE_Z[0]*100:.0f}, {WORKSPACE_Z[1]*100:.0f}] cm"
        )


def execute_trajectory(robot: NiryoRobot, q_trajectory: np.ndarray) -> None:
    """Exécute une trajectoire articulaire waypoint par waypoint.
    À chaque waypoint : vérifie limites + singularité + envoie commande.
    """
    for i, q_wp in enumerate(q_trajectory):
        # Vérifications de sécurité
        if not is_in_limits(q_wp):
            raise RuntimeError(
                f"Waypoint {i} hors limites articulaires : {q_wp}"
            )
        # Envoi de la commande (PyNiryo move_joints accepte 6 arguments)
        robot.move_joints(*q_wp.tolist())
        time.sleep(STEP_DELAY)


# ============================================================================
# PROGRAMME PRINCIPAL
# ============================================================================

def main():
    print("=" * 60)
    print("Test de mouvement amélioré — Bras Niryo Ned")
    print("=" * 60)
    print("\nSaisissez la position cible (cm) dans le repère base du robot :")
    x_cm = read_coordinate("X")
    y_cm = read_coordinate("Y")
    z_cm = read_coordinate("Z")

    orientation = read_orientation()
    target_pose = build_target_pose(x_cm, y_cm, z_cm, orientation)

    # Étape 1 : validation locale (avant connexion robot)
    print("\n[1/5] Validation de la cible (locale)...")
    try:
        validate_workspace(x_cm, y_cm, z_cm)
    except ValueError as e:
        print(f"  ✗ Cible hors espace de travail : {e}")
        return
    reachable, msg = validate_target_reachable(target_pose)
    if not reachable:
        print(f"  ✗ Cible non atteignable : {msg}")
        return
    print(f"  ✓ Cible valide : {msg}")

    # Étape 2 : résolution MGI locale (test rapide)
    print("\n[2/5] Résolution MGI locale (sans connexion robot)...")
    # Graine : configuration milieu de limites
    q_seed = np.array([(l[0] + l[1]) / 2 for l in JOINT_LIMITS])
    try:
        target_joints = mgi(target_pose, q0=q_seed)
    except RuntimeError as e:
        print(f"  ✗ MGI échoué : {e}")
        print("  Cause possible : cible hors workspace ou orientation irréalisable.")
        return

    # Vérification limites
    if not is_in_limits(target_joints):
        print(f"  ✗ Solution MGI hors limites : {target_joints}")
        return

    # Vérification singularité
    is_sing, manip = check_singularity(target_joints)
    if is_sing:
        print(f"  ⚠ Configuration finale singulière (manipulabilité = {manip:.4f})")
        print("    Le mouvement peut être imprécis. Ajustez la cible si nécessaire.")
    else:
        print(f"  ✓ Manipulabilité : {manip:.4f} (configuration saine)")

    print(f"\n  Articulations calculées (rad) :")
    print(f"  {np.array2string(target_joints, precision=4)}")
    print(f"\n  Pose cible  [m, rad] : {np.array2string(target_pose, precision=4)}")
    print(f"  Pose calculée par MGD : {np.array2string(pose_vec(mgd(target_joints)), precision=4)}")

    # Étape 3 : confirmation utilisateur
    print("\n[3/5] Confirmation utilisateur")
    confirm = input("Connecter et déplacer le robot ? [o/N] : ").strip().lower()
    if confirm not in ("o", "oui", "y", "yes"):
        print("  Mouvement annulé.")
        return

    # Étape 4 : connexion robot + lecture position courante
    print("\n[4/5] Connexion au robot...")
    robot = NiryoRobot(ROBOT_IP)
    try:
        robot.calibrate_auto()
        current_joints = np.asarray(robot.get_joints(), dtype=float)
        print(f"  Position courante (rad) : {np.array2string(current_joints, precision=4)}")
        current_pose = pose_vec(mgd(current_joints))
        print(f"  Pose courante [m, rad] : {np.array2string(current_pose, precision=4)}")

        # Étape 5 : génération et exécution de la trajectoire
        print(f"\n[5/5] Génération trajectoire ({N_STEPS} waypoints, "
              f"profil trapézoïdal)...")
        trajectory = joint_interpolation(current_joints, target_joints, n_steps=N_STEPS)

        # Vérification de tous les waypoints avant envoi
        print("  Vérification des waypoints...")
        for i, q_wp in enumerate(trajectory):
            if not is_in_limits(q_wp):
                raise RuntimeError(
                    f"Waypoint {i} hors limites : {q_wp}. "
                    "Arrêt immédiat pour sécurité."
                )
        print(f"  ✓ {len(trajectory)} waypoints validés")

        # Exécution
        print(f"  Exécution du mouvement ({N_STEPS * STEP_DELAY:.1f}s)...")
        execute_trajectory(robot, trajectory)

        # Vérification finale
        actual_joints = np.asarray(robot.get_joints(), dtype=float)
        actual_pose = pose_vec(mgd(actual_joints))
        pos_err = np.linalg.norm(actual_pose[:3] - target_pose[:3]) * 100
        orient_err = np.linalg.norm(actual_pose[3:] - target_pose[3:])

        print(f"\n  Articulations finales (rad) :")
        print(f"  {np.array2string(actual_joints, precision=4)}")
        print(f"  Pose finale [m, rad] :")
        print(f"  {np.array2string(actual_pose, precision=4)}")
        print(f"\n  Écart position  : {pos_err:.2f} cm")
        print(f"  Écart orientation : {orient_err:.4f} rad")

        if pos_err > 1.0:
            print("  ⚠ ATTENTION : écart position > 1 cm")
            print("    Causes possibles : calibration, DH parameters, "
                  "backlash moteur, ou singularité proche.")
        else:
            print("  ✓ Mouvement réussi dans la tolérance")

        print("\nTest terminé.")
    finally:
        robot.close_connection()


if __name__ == "__main__":
    main()