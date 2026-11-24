import sys
import os
import time
import argparse
import numpy as np

# Import du module cinématique
try:
    from Arm_kinematics import (
        mgd, mgi, pose_vec, pose_to_matrix, is_in_limits,
        validate_target_reachable, check_singularity, JOINT_LIMITS,
        _LOW, _HIGH
    )
except ImportError:
    print("ERREUR : Impossible d'importer Arm_kinematics.")
    print("Placez ce fichier dans le même dossier que Arm_kinematics.py")
    sys.exit(1)

# Import PyNiryo
try:
    from pyniryo import NiryoRobot
    PYNIRYO_AVAILABLE = True
except ImportError:
    PYNIRYO_AVAILABLE = False
    print("⚠ Module pyniryo non disponible — mode simulation (sans robot réel)")


# ============================================================================
# BACKEND CLAVIER (100% lettres, multiplateforme)
# ============================================================================

KEYBOARD_BACKEND = 'none'
if sys.platform == 'win32':
    try:
        import msvcrt
        KEYBOARD_BACKEND = 'msvcrt'
    except ImportError:
        pass
elif sys.platform in ('linux', 'linux2', 'darwin'):
    try:
        import termios, tty, select
        KEYBOARD_BACKEND = 'termios'
    except ImportError:
        pass


class KeyboardMsvcrt:
    """Windows : msvcrt (stdlib). Capture les lettres une par une."""
    def start(self): pass
    def stop(self): pass
    def get_last_key(self, timeout=0.0):
        if timeout > 0:
            deadline = time.time() + timeout
            while time.time() < deadline:
                if msvcrt.kbhit(): break
                time.sleep(0.005)
            else: return ''
        else:
            if not msvcrt.kbhit(): return ''
        ch = msvcrt.getch()
        b = ch[0] if isinstance(ch, bytes) else ord(ch)
        if b == 3: return 'ctrl_c'
        if b == 27: return 'esc'
        try: return chr(b).lower()
        except ValueError: return ''


class KeyboardTermios:
    """Unix : termios/tty (stdlib)."""
    def __init__(self):
        self._fd = None
        self._old = None
    def start(self):
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
    def stop(self):
        if self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            self._old = None
    def get_last_key(self, timeout=0.0):
        if timeout <= 0:
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if not r: return ''
        else:
            deadline = time.time() + timeout
            while time.time() < deadline:
                r, _, _ = select.select([sys.stdin], [], [], 0.005)
                if r: break
            else: return ''
        ch = sys.stdin.read(1)
        if ch == '\x03': return 'ctrl_c'
        if ch == '\x1b': return 'esc'
        return ch.lower()


class CommandLineReader:
    """Fallback : lecture ligne par ligne."""
    def start(self): pass
    def stop(self): pass
    def get_last_key(self, timeout=0.0):
        try:
            line = input("> ").strip().lower()
            if not line: return ''
            if line in ('quit', 'exit'): return 'x'
            return line[0]
        except EOFError:
            return 'x'


def create_keyboard():
    if KEYBOARD_BACKEND == 'msvcrt': return KeyboardMsvcrt()
    if KEYBOARD_BACKEND == 'termios': return KeyboardTermios()
    return CommandLineReader()


# ============================================================================
# CONFIGURATION
# ============================================================================

DEFAULT_IP = "10.10.3.156"
DEFAULT_STEP_CM = 1.0
STEP_RAD_FACTOR = 0.1
ROBOT_VELOCITY = 80
DEFAULT_GRIPPER_SPEED = 500

MAX_REACH_RADIUS = 0.48
MIN_REACH_RADIUS = 0.10
SAFETY_MARGIN = 0.95

HOME_JOINTS = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

WORKSPACE_LIMITS = {
    'x': (0.05, 0.45),
    'y': (-0.30, 0.30),
    'z': (0.05, 0.50),
}


# ============================================================================
# SIMULATEUR (mode sans robot)
# ============================================================================

class SimulatedRobot:
    """Simule un robot Niryo pour tester sans matériel."""
    def __init__(self):
        self._q = HOME_JOINTS.copy()
        self._gripper_open = False
        print("  [Simulateur] Robot initialisé à la position home")
    def calibrate_auto(self):
        print("  [Simulateur] Calibration auto (simulée)")
    def set_arm_max_velocity(self, v): pass
    def get_joints(self): return self._q.tolist()
    def move_joints(self, *joints):
        self._q = np.asarray(joints, dtype=float)
        time.sleep(0.02)
    def open_gripper(self, speed=500, **kwargs):
        self._gripper_open = True
        print(f"  [Simulateur] Pince OUVERTE (vitesse {speed})")
    def close_gripper(self, speed=500, **kwargs):
        self._gripper_open = False
        print(f"  [Simulateur] Pince FERMÉE (vitesse {speed})")
    def grasp_with_tool(self, *args, **kwargs):
        self._gripper_open = False
        print("  [Simulateur] SAISIE auto — pince fermée sur objet")
        return True
    def release_grasp_with_tool(self, *args, **kwargs):
        self._gripper_open = True
        print("  [Simulateur] RELÂCHÉ — pince ouverte")
    def get_tool_state(self):
        return 100 if self._gripper_open else 0
    def update_tool(self, *args, **kwargs):
        pass
    def close_connection(self):
        print("  [Simulateur] Connexion fermée")


# ============================================================================
# CONTRÔLEUR DU BRAS — synchrone, MGI + move_joints (version v5 qui marchait)
# ============================================================================

class ArmController:
    """Contrôleur synchrone avec MGI Python + move_joints.

    - target_pose accumulée avec projection workspace
    - MGI résolue sur la POSE COMPLÈTE (axes couplés)
    - move_joints direct (trajectoire interne Niryo)
    - Dispatcher multi-signatures pour la pince
    """

    def __init__(self, robot, step_cm, gripper_speed=DEFAULT_GRIPPER_SPEED):
        self.robot = robot
        self.step_m = step_cm / 100.0
        self.step_rad = step_cm * STEP_RAD_FACTOR
        self.gripper_speed = gripper_speed

        robot.calibrate_auto()
        try:
            robot.set_arm_max_velocity(ROBOT_VELOCITY)
        except Exception:
            pass

        # Détection de l'effecteur
        self.tool_detected = False
        try:
            if hasattr(robot, 'update_tool'):
                robot.update_tool()
                self.tool_detected = True
        except Exception:
            pass

        # État initial de la pince
        self.gripper_state = 'unknown'
        try:
            tool_state = robot.get_tool_state()
            self.gripper_state = 'open' if tool_state > 50 else 'closed'
        except Exception:
            self.gripper_state = 'closed'

        # État courant
        self.current_q = np.asarray(robot.get_joints(), dtype=float)
        self.current_pose = pose_vec(mgd(self.current_q))
        self.target_pose = self.current_pose.copy()

        # Stats
        self.last_error = ""
        self.last_error_time = 0
        self.success_count = 0
        self.fail_count = 0

    # ----------------------------------------------------------------------
    # Mise à jour cible (avec projection workspace)
    # ----------------------------------------------------------------------

    def add_delta(self, dx=0, dy=0, dz=0, droll=0, dpitch=0, dyaw=0):
        """Ajoute delta à la cible + projette sur workspace. NON bloquant.
        Retourne (success, message).
        """
        new_target = self.target_pose.copy()
        new_target[0] += dx
        new_target[1] += dy
        new_target[2] += dz
        new_target[3] += droll
        new_target[4] += dpitch
        new_target[5] += dyaw

        # Clamp Z
        z_min, z_max = WORKSPACE_LIMITS['z']
        if new_target[2] < z_min:
            new_target[2] = z_min
        elif new_target[2] > z_max:
            new_target[2] = z_max

        # Projection radiale : si r > max_reach, scale X/Y proportionnellement
        x, y = new_target[0], new_target[1]
        r = np.hypot(x, y)
        max_r = MAX_REACH_RADIUS * SAFETY_MARGIN

        adjusted_msg = ""
        if r > max_r:
            scale = max_r / r
            new_target[0] = x * scale
            new_target[1] = y * scale
            adjusted_msg = f"Rayon limité : X/Y réduit à {scale*100:.0f}%"
        elif r < MIN_REACH_RADIUS and new_target[2] < 0.20:
            if r < 1e-6:
                new_target[0] = MIN_REACH_RADIUS
                new_target[1] = 0.0
            else:
                scale = MIN_REACH_RADIUS / r
                new_target[0] = x * scale
                new_target[1] = y * scale
            adjusted_msg = f"Distance base augmentée à {MIN_REACH_RADIUS:.3f}m"

        self.target_pose = new_target
        return True, adjusted_msg

    # ----------------------------------------------------------------------
    # Exécution du mouvement (MGI sur pose complète + move_joints)
    # ----------------------------------------------------------------------

    def execute_move(self):
        """Exécute le mouvement vers target_pose. SYNCHRONE (bloquant).
        Retourne (success, message, diagnostic).
        """
        delta = self.target_pose - self.current_pose
        pos_err = np.linalg.norm(delta[:3])
        orient_err = np.linalg.norm(delta[3:])

        if pos_err < 1e-4 and orient_err < 1e-3:
            return True, "no move", ""

        # Validation reachability
        reachable, msg = validate_target_reachable(self.target_pose)
        if not reachable:
            self.fail_count += 1
            self.last_error = msg
            self.last_error_time = time.time()
            self.target_pose = self.current_pose.copy()
            return False, f"Non atteignable : {msg}", ""

        # Vérification singularité
        is_sing, manip = check_singularity(self.current_q)
        if is_sing:
            self.fail_count += 1
            self.last_error = f"singularité (w={manip:.4f})"
            self.last_error_time = time.time()
            return False, f"Config singulière (w={manip:.4f})", ""

        # MGI sur la POSE COMPLÈTE
        try:
            target_q = mgi(self.target_pose, q0=self.current_q)
        except RuntimeError as e:
            self.fail_count += 1
            self.last_error = str(e)
            self.last_error_time = time.time()
            self.target_pose = self.current_pose.copy()
            return False, f"MGI : {e}", ""

        # Vérification limites articulaires
        if not is_in_limits(target_q):
            self.fail_count += 1
            self.last_error = "limites articulaires"
            self.last_error_time = time.time()
            self.target_pose = self.current_pose.copy()
            violated = []
            for i, (qv, (lo, hi)) in enumerate(zip(target_q, JOINT_LIMITS)):
                if qv < lo or qv > hi:
                    violated.append(f"q{i+1}={qv:.3f}")
            return False, f"Hors limites : {', '.join(violated)}", ""

        # MOUVEMENT : move_joints direct
        try:
            self.robot.move_joints(*target_q.tolist())
        except Exception as e:
            self.fail_count += 1
            self.last_error = str(e)
            self.last_error_time = time.time()
            return False, f"Erreur robot : {e}", ""

        # Mise à jour état
        self.current_q = np.asarray(self.robot.get_joints(), dtype=float)
        self.current_pose = pose_vec(mgd(self.current_q))
        self.target_pose = self.current_pose.copy()
        self.success_count += 1
        return True, "OK", ""

    # ----------------------------------------------------------------------
    # Actions spéciales
    # ----------------------------------------------------------------------

    def move_home(self):
        """Retour home. SYNCHRONE. Retourne (success, message)."""
        try:
            self.robot.move_joints(*HOME_JOINTS.tolist())
        except Exception as e:
            self.fail_count += 1
            self.last_error = str(e)
            self.last_error_time = time.time()
            return False, f"Erreur robot : {e}"
        self.current_q = HOME_JOINTS.copy()
        self.current_pose = pose_vec(mgd(self.current_q))
        self.target_pose = self.current_pose.copy()
        self.success_count += 1
        return True, "Home atteint"

    def reset_target_to_current(self):
        """Reset target → current."""
        self.target_pose = self.current_pose.copy()
        return True, "Cible reset vers pose courante"

    # ----------------------------------------------------------------------
    # DISPATCHER PINCE — multi-signatures (compatible toutes versions PyNiryo)
    # ----------------------------------------------------------------------

    def _call_gripper_method(self, method_name, speed):
        """Appelle open_gripper/close_gripper en s'adaptant à la version PyNiryo.

        Essaie plusieurs signatures :
          - method(speed, wait=True)   → PyNiryo récent
          - method(speed)              → PyNiryo moyen
          - method(speed_open=speed)   → variante
          - method()                   → ancien
        """
        method = getattr(self.robot, method_name, None)
        if method is None:
            return False, f"Méthode {method_name} inexistante"

        attempts = [
            ("speed + wait", lambda: method(speed, wait=True)),
            ("speed only",   lambda: method(speed)),
            ("speed_open",   lambda: method(speed_open=speed)),
            ("speed_close",  lambda: method(speed_close=speed)),
            ("no args",      lambda: method()),
        ]

        for label, attempt in attempts:
            try:
                attempt()
                return True, label
            except TypeError:
                continue
            except Exception as e:
                return False, f"{e} (tentative: {label})"
        return False, "Aucune signature compatible"

    def open_gripper(self):
        """Ouvre la pince. Retourne (success, message)."""
        success, msg = self._call_gripper_method('open_gripper', self.gripper_speed)
        if success:
            self.gripper_state = 'open'
            self.success_count += 1
            return True, f"Pince OUVERTE ({msg})"
        self.fail_count += 1
        self.last_error = msg
        self.last_error_time = time.time()
        return False, f"Erreur ouverture : {msg}"

    def close_gripper(self):
        """Ferme la pince. Retourne (success, message)."""
        success, msg = self._call_gripper_method('close_gripper', self.gripper_speed)
        if success:
            self.gripper_state = 'closed'
            self.success_count += 1
            return True, f"Pince FERMÉE ({msg})"
        self.fail_count += 1
        self.last_error = msg
        self.last_error_time = time.time()
        return False, f"Erreur fermeture : {msg}"

    def grasp_object(self):
        """Saisie auto. Fallback : close_gripper."""
        grasp = getattr(self.robot, 'grasp_with_tool', None)
        if grasp is not None:
            try:
                grasp()
                self.gripper_state = 'holding'
                self.success_count += 1
                return True, "SAISIE auto — objet tenu"
            except Exception as e:
                self.last_error = f"grasp_with_tool: {e}"
                self.last_error_time = time.time()

        # Fallback : close_gripper
        success, msg = self._call_gripper_method('close_gripper', self.gripper_speed)
        if success:
            self.gripper_state = 'holding'
            self.success_count += 1
            return True, f"Saisie (fallback close, {msg})"
        self.fail_count += 1
        return False, f"Erreur saisie : {msg}"

    def release_object(self):
        """Relâche la pince."""
        release = getattr(self.robot, 'release_grasp_with_tool', None)
        if release is not None:
            try:
                release()
                self.gripper_state = 'open'
                self.success_count += 1
                return True, "RELÂCHÉ — pince ouverte"
            except Exception as e:
                self.last_error = f"release_grasp: {e}"
                self.last_error_time = time.time()

        success, msg = self._call_gripper_method('open_gripper', self.gripper_speed)
        if success:
            self.gripper_state = 'open'
            self.success_count += 1
            return True, f"Relâché (fallback open, {msg})"
        self.fail_count += 1
        return False, f"Erreur relâchement : {msg}"

    # ----------------------------------------------------------------------
    # Affichage
    # ----------------------------------------------------------------------

    def display_pose(self):
        """Affiche pose courante + cible + état pince."""
        cp = self.current_pose
        tp = self.target_pose
        diff = np.linalg.norm(tp[:3] - cp[:3])
        indicator = "✓" if diff < 0.001 else "→"
        status = "OK" if diff < 0.001 else f"Δ={diff*100:.1f}cm"
        grip_sym = {'open': '○', 'closed': '●', 'holding': '◉'}.get(self.gripper_state, '?')
        print(f"\r  {indicator} Courant: X={cp[0]*100:6.2f} Y={cp[1]*100:6.2f} "
              f"Z={cp[2]*100:6.2f}cm R={np.degrees(cp[3]):5.1f}° "
              f"P={np.degrees(cp[4]):5.1f}° Y={np.degrees(cp[5]):5.1f}° "
              f"| Cible: X={tp[0]*100:6.2f} Y={tp[1]*100:6.2f} Z={tp[2]*100:6.2f}cm "
              f"[{status}] Pince:{grip_sym}    ",
              end='', flush=True)

    def display_debug(self):
        """Affiche l'état complet pour debug."""
        cp = self.current_pose
        tp = self.target_pose
        cq = self.current_q
        r_cur = np.hypot(cp[0], cp[1])
        r_tgt = np.hypot(tp[0], tp[1])
        max_r = MAX_REACH_RADIUS * SAFETY_MARGIN
        is_sing, manip = check_singularity(cq)

        print("\n" + "=" * 70)
        print("  DEBUG — État complet")
        print("=" * 70)
        print(f"  Effecteur détecté : {'oui' if self.tool_detected else 'non'}")
        print(f"\n  Articulations courantes (rad) :")
        for i, (qv, (lo, hi)) in enumerate(zip(cq, JOINT_LIMITS)):
            in_lim = "✓" if lo <= qv <= hi else "✗"
            print(f"    q{i+1} = {qv:+.4f}  (limite [{lo:+.3f}, {hi:+.3f}]) {in_lim}")
        print(f"\n  Pose courante :")
        print(f"    Position  : X={cp[0]*100:6.2f} Y={cp[1]*100:6.2f} Z={cp[2]*100:6.2f} cm")
        print(f"    Orientation : R={np.degrees(cp[3]):6.2f}° P={np.degrees(cp[4]):6.2f}° Y={np.degrees(cp[5]):6.2f}°")
        print(f"    Rayon radial : {r_cur*100:.2f} cm (max {max_r*100:.2f} cm)")
        print(f"\n  Cible :")
        print(f"    Position  : X={tp[0]*100:6.2f} Y={tp[1]*100:6.2f} Z={tp[2]*100:6.2f} cm")
        print(f"    Rayon radial : {r_tgt*100:.2f} cm")
        print(f"\n  Manipulabilité : w={manip:.4f} ({'SINGULIÈRE' if is_sing else 'OK'})")
        print(f"  Pince : état={self.gripper_state}, vitesse={self.gripper_speed}")
        print(f"  Stats : {self.success_count} succès, {self.fail_count} échecs")
        if self.last_error and (time.time() - self.last_error_time < 30):
            print(f"  Dernière erreur : {self.last_error}")
        print("=" * 70)


# ============================================================================
# AIDE
# ============================================================================

HELP_TEXT = """
╔══════════════════════════════════════════════════════════════╗
║   CONTRÔLE CLAVIER — Bras Niryo Ned                         ║
╠══════════════════════════════════════════════════════════════╣
║   POSITION (XYZ)                                             ║
║     i / k       : +X / -X   (avant / arrière)                ║
║     j / l       : -Y / +Y   (gauche / droite)                ║
║     u / o       : +Z / -Z   (haut / bas)                     ║
║                                                              ║
║   ORIENTATION (RPY)                                          ║
║     w / s       : -pitch / +pitch                            ║
║     a / d       : -roll / +roll                              ║
║     q / e       : -yaw / +yaw                                ║
║                                                              ║
║   EFFECTEUR (pince)                                          ║
║     g           : OUVRIR la pince                            ║
║     f           : FERMER la pince                            ║
║     b           : saisir avec détection auto (grasp)         ║
║     n           : relâcher la saisie (release)               ║
║                                                              ║
║   ACTIONS                                                    ║
║     r           : retour home                                ║
║     z           : RESET cible → pose courante                ║
║     n           : DEBUG (état complet)                       ║
║     h           : aide                                       ║
║     c           : pose courante                              ║
║     x           : quitter                                    ║
╠══════════════════════════════════════════════════════════════╣
║   Incrément : {step_cm:.1f} cm  |  Vitesse robot : {velocity}%          ║
║   Vitesse pince : {grip_speed}                                     ║
║                                                              ║
║   Symboles pince : ○ ouverte  ● fermée  ◉ objet tenu        ║
╚══════════════════════════════════════════════════════════════╝
""".strip()

def display_help(step_cm, velocity, grip_speed):
    print("\n" + HELP_TEXT.format(step_cm=step_cm, velocity=velocity, grip_speed=grip_speed))


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Pilotage temps réel du bras Niryo"
    )
    parser.add_argument('--ip', default=DEFAULT_IP)
    parser.add_argument('--step', type=float, default=DEFAULT_STEP_CM)
    parser.add_argument('--gripper-speed', type=int, default=DEFAULT_GRIPPER_SPEED)
    args = parser.parse_args()

    step_rad = args.step * STEP_RAD_FACTOR

    print("=" * 64)
    print("  Pilotage temps réel du bras Niryo Ned")
    print("  MGI + move_joints | Effecteur (pince) | Synchrone")
    print("=" * 64)
    print(f"\n  IP robot : {args.ip}")
    print(f"  Incrément : {args.step:.2f} cm / {step_rad:.3f} rad")
    print(f"  Vitesse robot : {ROBOT_VELOCITY}% | Vitesse pince : {args.gripper_speed}")
    print(f"  Backend clavier : {KEYBOARD_BACKEND}")

    print(f"\n[1] Connexion...")
    if PYNIRYO_AVAILABLE:
        try:
            robot = NiryoRobot(args.ip)
        except Exception as e:
            print(f"  ✗ Connexion échouée : {e} → simulation")
            robot = SimulatedRobot()
    else:
        robot = SimulatedRobot()

    print(f"\n[2] Init contrôleur...")
    try:
        controller = ArmController(robot, args.step, gripper_speed=args.gripper_speed)
    except Exception as e:
        print(f"  ✗ Erreur : {e}")
        robot.close_connection()
        return

    print(f"  ✓ Prêt — effecteur {'détecté' if controller.tool_detected else 'non détecté'}")
    display_help(args.step, ROBOT_VELOCITY, args.gripper_speed)
    print(f"\n  Position initiale :")
    controller.display_pose()
    print(f"\n\n  → i/k/j/l/u/o déplacer | w/s/a/d/q/e orienter | g/f/b/n pince")
    print(f"    'z' = reset | 'd' = debug | 'r' = home | 'x' = quit\n")

    kb = create_keyboard()
    try:
        kb.start()
    except Exception as e:
        print(f"  ⚠ Backend échoué ({e}) → mode ligne")
        kb = CommandLineReader()
        kb.start()

    KEY_MAP = {
        'i': dict(dx=+controller.step_m),
        'k': dict(dx=-controller.step_m),
        'j': dict(dy=-controller.step_m),
        'l': dict(dy=+controller.step_m),
        'u': dict(dz=+controller.step_m),
        'o': dict(dz=-controller.step_m),
        'w': dict(dpitch=-step_rad),
        's': dict(dpitch=+step_rad),
        'a': dict(droll=-step_rad),
        'n': dict(droll=+step_rad),
        'q': dict(dyaw=-step_rad),
        'e': dict(dyaw=+step_rad),
    }

    should_quit = False

    try:
        while not should_quit:
            key = kb.get_last_key(timeout=0.1)

            if not key:
                continue

            # Actions spéciales
            if key in ('x', 'esc', 'ctrl_c'):
                print("\n  Arrêt demandé.")
                should_quit = True
                break
            elif key == 'h':
                display_help(args.step, ROBOT_VELOCITY, args.gripper_speed)
                continue
            elif key == 'c':
                controller.display_pose()
                print()
                continue
            elif key == 'd':
                controller.display_debug()
                continue
            elif key == 'r':
                print("\n  → Retour home...", end='', flush=True)
                success, msg = controller.move_home()
                print(f" {'✓' if success else '✗'} {msg}")
                controller.display_pose()
                print()
                continue
            elif key == 'z':
                success, msg = controller.reset_target_to_current()
                print(f"\n  ↺ {msg}")
                controller.display_pose()
                print()
                continue
            # --- Effecteur (pince) ---
            elif key == 'g':
                print("\n  → Ouverture pince...", end='', flush=True)
                success, msg = controller.open_gripper()
                print(f" {'✓' if success else '✗'} {msg}")
                controller.display_pose()
                print()
                continue
            elif key == 'f':
                print("\n  → Fermeture pince...", end='', flush=True)
                success, msg = controller.close_gripper()
                print(f" {'✓' if success else '✗'} {msg}")
                controller.display_pose()
                print()
                continue
            elif key == 'b':
                print("\n  → Saisie auto...", end='', flush=True)
                success, msg = controller.grasp_object()
                print(f" {'✓' if success else '✗'} {msg}")
                controller.display_pose()
                print()
                continue
            elif key == 'n':
                print("\n  → Relâchement...", end='', flush=True)
                success, msg = controller.release_object()
                print(f" {'✓' if success else '✗'} {msg}")
                controller.display_pose()
                print()
                continue
            # --- Mouvement ---
            elif key in KEY_MAP:
                # 1. Mettre à jour la cible (avec projection workspace)
                controller.add_delta(**KEY_MAP[key])
                # 2. Exécuter le mouvement (MGI + move_joints)
                success, msg, diag = controller.execute_move()
                if success:
                    controller.display_pose()
                else:
                    print(f"\n  ⚠ ÉCHEC : {msg}")
                    controller.display_pose()
                    time.sleep(0.2)
            else:
                continue

    except KeyboardInterrupt:
        print("\n  Ctrl-C...")
    except Exception as e:
        print(f"\n  ✗ Erreur fatale : {e}")
        import traceback
        traceback.print_exc()
    finally:
        kb.stop()
        print(f"\n\n[Fin] Fermeture...")
        robot.close_connection()
        try:
            print(f"  Position finale :")
            controller.display_pose()
            print()
            print(f"  Stats : {controller.success_count} succès, "
                  f"{controller.fail_count} échecs")
        except Exception:
            pass
        print("=" * 64)
        print("  Fin du programme.")
        print("=" * 64)


if __name__ == "__main__":
    main()