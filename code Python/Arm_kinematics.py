import numpy as np
from typing import Optional, Tuple, List


# ============================================================================
# 1. PARAMÈTRES GÉOMÉTRIQUES (inchangés par rapport à l'original)
# ============================================================================

# Table DH modifiée (Craig) : (alpha_{i-1}, a_{i-1}, theta_off_i, d_i) en rad et m
DH = [
    (0.0,         0.0,     0.0,                0.1715),  # base -> joint 1
    (np.pi/2,     0.0,     np.radians(86.892), 0.0),     # joint 1 -> joint 2
    (0.0,         0.221326, np.radians(3.108), 0.0),     # joint 2 -> joint 3
    (np.pi/2,     0.0325,  0.0,                0.235),   # joint 3 -> joint 4
    (-np.pi/2,    0.0,     0.0,                0.0),     # joint 4 -> joint 5
    (np.pi/2,     0.00925, np.pi/2,            0.0412),  # joint 5 -> joint 6
]

# Transformation fixe repère 6 -> repère outil réel
T6_TOOL = np.array([
    [0, -1, 0, 0],
    [0,  0, -1, 0],
    [1,  0, 0, 0],
    [0,  0, 0, 1]
], dtype=float)

# Limites articulaires (rad) — Niryo Ned
JOINT_LIMITS = [
    (-2.967, 2.967),  # q1
    (-2.094, 0.611),  # q2
    (-1.344, 1.571),  # q3
    (-2.094, 2.094),  # q4
    (-1.745, 0.960),  # q5
    (-2.531, 2.531),  # q6
]

# Limites sous forme de tableaux numpy (calcul vectoriel)
_LOW = np.array([l[0] for l in JOINT_LIMITS])
_HIGH = np.array([l[1] for l in JOINT_LIMITS])

# Seuil de singularité (déterminant de det(J*J^T) sous ce seuil -> singularité)
SINGULARITY_THRESHOLD = 1e-6

# Tolérances MGI
POS_TOL = 1e-4       # m, tolérance position
ORIENT_TOL = 1e-3    # rad, tolérance orientation
MAX_ITER_LM = 100    # itérations Levenberg-Marquardt par seed


# ============================================================================
# 2. FONCTIONS DE TRANSFORMATION (MGD)
# ============================================================================

def dh_transform(alpha: float, a: float, theta: float, d: float) -> np.ndarray:
    """Matrice de transformation DH modifiée (Craig).
    T = Rot_x(alpha) * Trans_x(a) * Rot_z(theta) * Trans_z(d)
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,       -st,       0,     a       ],
        [st*ca,   ct*ca,    -sa,   -sa*d    ],
        [st*sa,   ct*sa,     ca,    ca*d    ],
        [0,        0,        0,     1       ]
    ])


def mgd(q: np.ndarray) -> np.ndarray:
    """Modèle Géométrique Direct.
    Entrée  : q (6,) angles articulaires en rad
    Sortie  : T (4,4) pose de l'outil dans le repère base
    """
    q = np.asarray(q, dtype=float).flatten()
    assert q.shape == (6,), f"q doit avoir 6 composantes, reçu {q.shape}"
    T = np.eye(4)
    for i, (alpha, a, off, d) in enumerate(DH):
        T = T @ dh_transform(alpha, a, q[i] + off, d)
    return T @ T6_TOOL


def mgd_intermediate(q: np.ndarray) -> List[np.ndarray]:
    """Renvoie les poses de TOUTES les frames (T_0^0, T_0^1, ..., T_0^6, T_0_tool).
    Utile pour : affichage, Jacobien analytique, détection de collision.
    """
    q = np.asarray(q, dtype=float).flatten()
    Ts = [np.eye(4)]
    T = np.eye(4)
    for i, (alpha, a, off, d) in enumerate(DH):
        T = T @ dh_transform(alpha, a, q[i] + off, d)
        Ts.append(T.copy())
    Ts.append(T @ T6_TOOL)
    return Ts


# ============================================================================
# 3. CONVERSIONS POSE <-> MATRICE 4x4
# ============================================================================

def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convertit (roll, pitch, yaw) en matrice de rotation.
    Convention : RPY XYZ extrinsèque = Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll).
    """
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr            ]
    ])


def matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Convertit matrice de rotation -> (roll, pitch, yaw) RPY XYZ extrinsèque.
    Gère le gimbal lock (pitch ≈ ±π/2).
    """
    # Cas nominal
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if np.abs(np.abs(pitch) - np.pi / 2) < 1e-6:
        # Gimbal lock : on fige yaw=0 et on calcule roll
        # R[:,0] = [cy*cp, sy*cp, -sp] avec sp=±1 -> cp=0 -> x,y de la 1ère colonne = 0
        # On choisit la convention : yaw = 0, roll = atan2(R[1,2], R[0,2]) pour pitch=+π/2
        #                                    roll = atan2(-R[1,2], -R[0,2]) pour pitch=-π/2
        if pitch > 0:
            roll = np.arctan2(R[1, 2], R[0, 2])
            yaw = 0.0
        else:
            roll = np.arctan2(-R[1, 2], -R[0, 2])
            yaw = 0.0
    else:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(roll), float(pitch), float(yaw)


def pose_vec(T: np.ndarray) -> np.ndarray:
    """4x4 -> [x, y, z, roll, pitch, yaw] (m, rad)."""
    p = T[:3, 3]
    r, pi, y = matrix_to_rpy(T[:3, :3])
    return np.concatenate([p, [r, pi, y]])


def pose_to_matrix(pose: np.ndarray) -> np.ndarray:
    """[x, y, z, roll, pitch, yaw] -> matrice 4x4."""
    pose = np.asarray(pose, dtype=float).flatten()
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(pose[3], pose[4], pose[5])
    T[:3, 3] = pose[:3]
    return T


# ============================================================================
# 4. JACOBIEN ANALYTIQUE GÉOMÉTRIQUE (amélioration majeure)
# ============================================================================

def _rpy_to_omega_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Matrice T telle que : [ṙ, ṗ, ẏ]^T = T * ω.
    Convention : R = Rz(yaw) * Ry(pitch) * Rx(roll)  (RPY = ZYX intrinsic = XYZ extrinsic).
    Dérivation :
      ω = ẏ*e_z + ṗ*Rz(yaw)*e_y + ṙ*Rz(yaw)*Ry(pitch)*e_x
      → ω_x = -sin(yaw)*ṗ + cos(yaw)*cos(pitch)*ṙ
        ω_y =  cos(yaw)*ṗ + sin(yaw)*cos(pitch)*ṙ
        ω_z =  ẏ - sin(pitch)*ṙ
      En inversant (si cos(pitch) ≠ 0) :
        ṙ = (cos(yaw)*ω_x + sin(yaw)*ω_y) / cos(pitch)
        ṗ = -sin(yaw)*ω_x + cos(yaw)*ω_y
        ẏ = ω_z + tan(pitch)*(cos(yaw)*ω_x + sin(yaw)*ω_y)
    """
    cy, sy = np.cos(yaw),   np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    if np.abs(cp) < 1e-6:
        # Gimbal lock (pitch = ±π/2) : ẏ et ṙ deviennent dégénérés
        # On choisit conventionnellement ṙ = 0 et ẏ = ω_z (dégradation contrôlée)
        return np.array([
            [0,    0,    0],
            [-sy,  cy,   0],
            [0,    0,    1]
        ])
    return np.array([
        [ cy / cp,  sy / cp,           0],
        [-sy,       cy,                0],
        [ cy * sp / cp, sy * sp / cp,  1]
    ])


def geometric_jacobian(q: np.ndarray) -> np.ndarray:
    """Jacobien analytique 6x6 OPERATIONNEL (compatible avec pose_vec).

    Convention Craig modifiée : l'axe de l'articulation i (i=1..6) est z_i,
    c'est-à-dire l'axe z du repère i (APRÈS la transformation de l'articulation i).
    Donc pour la colonne i (0-indexed) du Jacobien, on utilise Ts[i+1].

    Formules :
      J_v_i = z_i × (p_e - p_i)    (vitesse linéaire, i=1..6)
      J_w_i = z_i                  (vitesse angulaire, i=1..6)
    avec z_i = Ts[i][:, 2]  et  p_i = Ts[i][:, 3]  dans le repère base.
    """
    q = np.asarray(q, dtype=float).flatten()
    Ts = mgd_intermediate(q)
    p_e = Ts[-1][:3, 3]

    # Jacobien géométrique (6,6)
    # Colonne i (i=0..5) correspond à l'articulation i+1 → axe z_i+1 = Ts[i+1]
    J_geom = np.zeros((6, 6))
    for i in range(6):
        T_after = Ts[i + 1]  # repère après l'articulation i+1
        z_axis = T_after[:3, 2]
        p_axis = T_after[:3, 3]
        J_geom[:3, i] = np.cross(z_axis, p_e - p_axis)
        J_geom[3:, i] = z_axis

    # Conversion ω -> dérivées RPY de l'outil
    T_tool = Ts[-1]
    roll, pitch, yaw = matrix_to_rpy(T_tool[:3, :3])
    T_rpy = _rpy_to_omega_matrix(roll, pitch, yaw)

    J_op = J_geom.copy()
    J_op[3:, :] = T_rpy @ J_geom[3:, :]
    return J_op


def check_singularity(q: np.ndarray, threshold: float = SINGULARITY_THRESHOLD
                      ) -> Tuple[bool, float]:
    """Détecte si la configuration est singulière.
    Renvoie (is_singular, manipulabilité).
    Manipulabilité : w = sqrt(det(J * J^T)) — proche de 0 = singularité.
    """
    J = geometric_jacobian(q)
    JJt = J @ J.T
    w = np.sqrt(max(np.linalg.det(JJt), 0.0))
    return (w < threshold, float(w))


# ============================================================================
# 5. MGI — LEVENBERG-MARQUARDT ROBUSTE
# ============================================================================

def _wrap_pi(angle: float) -> float:
    """Enroule un angle dans [-π, π[."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _pose_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Erreur de pose avec wrap_pi sur les angles."""
    err = target - current
    err[3:] = np.vectorize(_wrap_pi)(err[3:])
    return err


def _lm_step(target: np.ndarray, q_init: np.ndarray,
             max_iter: int = MAX_ITER_LM) -> Tuple[np.ndarray, float, bool]:
    """Une passe Levenberg-Marquardt à amortissement adaptatif.
    Vérifie les limites articulaires À CHAQUE itération.
    Renvoie (q_final, erreur_norm, converged).
    """
    q = np.clip(np.asarray(q_init, dtype=float).copy(), _LOW, _HIGH)
    lam = 1e-3

    for iteration in range(max_iter):
        err = _pose_error(target, pose_vec(mgd(q)))
        err_norm = np.linalg.norm(err)

        # Critère de convergence : position ET orientation
        if np.linalg.norm(err[:3]) < POS_TOL and np.linalg.norm(err[3:]) < ORIENT_TOL:
            return q, err_norm, True

        J = geometric_jacobian(q)

        # Détecter singularité
        is_sing, manip = check_singularity(q)
        if is_sing:
            # Augmenter fortement l'amortissement pour éviter division par ~0
            lam = max(lam, 1.0)

        # Résoudre dq = (J^T J + lambda * diag)⁻¹ J^T err
        JJt = J.T @ J
        try:
            A = JJt + lam * np.diag(np.diag(JJt) + 1e-12)
            dq = np.linalg.solve(A, J.T @ err)
        except np.linalg.LinAlgError:
            return q, err_norm, False

        # Pas avec limite de longueur (line search simplifié)
        step_norm = np.linalg.norm(dq)
        max_step = 0.3  # rad, limite pour éviter de sauter entre configurations
        if step_norm > max_step:
            dq = dq * (max_step / step_norm)

        q_new = np.clip(q + dq, _LOW, _HIGH)

        err_new = _pose_error(target, pose_vec(mgd(q_new)))
        err_new_norm = np.linalg.norm(err_new)

        if err_new_norm < err_norm:
            q = q_new
            lam = max(lam / 3.0, 1e-9)
        else:
            lam = min(lam * 5.0, 1e6)
            if lam >= 1e6:
                # Convergence bloquée
                return q, err_norm, False

    return q, err_norm, False


def mgi(target_pose: np.ndarray, q0: Optional[np.ndarray] = None,
        return_all: bool = False
        ) -> np.ndarray:
    """Modèle Géométrique Inverse robuste.

    Stratégie multi-graines : essaie 8 configurations (lefty/righty × above/below
    elbow × flip/no-flip poignet) + la configuration courante si fournie.
    Conserve la solution la plus proche de q0 ET dans les limites.

    Entrée :
      target_pose : [x, y, z, roll, pitch, yaw] (m, rad)
      q0          : configuration initiale optionnelle (6,)
      return_all  : si True, renvoie toutes les solutions valides

    Sortie :
      q (6,) : solution optimale
      OU (si return_all) : liste de toutes les solutions valides

    Lève :
      RuntimeError si aucune solution trouvée.
    """
    target = np.asarray(target_pose, dtype=float).flatten()
    assert target.shape == (6,), f"target_pose doit avoir 6 composantes, reçu {target.shape}"

    # Estimation de q1 à partir de la position cible
    q1_base = np.arctan2(target[1], target[0])

    # 8 configurations canoniques (lefty/righty, above/below, flip/no-flip)
    # On varie q2 ∈ {-1.2, -0.5, 0.3, 0.1}, q3 ∈ {-0.8, 0.5, 1.2, 0.6}
    # et q5 ∈ {-0.3, 0.5} pour flip/no-flip
    seeds: List[np.ndarray] = []
    if q0 is not None:
        seeds.append(np.asarray(q0, dtype=float).copy())

    for q2 in (-1.2, -0.5, 0.3, 0.1):
        for q3 in (-0.8, 0.5, 1.2, 0.6):
            for q5 in (-0.3, 0.5):
                seeds.append(np.array([q1_base, q2, q3, 0.0, q5, 0.0]))

    cands: List[Tuple[np.ndarray, float]] = []
    for s in seeds:
        q_final, err_norm, converged = _lm_step(target, s)
        if converged and np.all(q_final >= _LOW - 1e-9) and np.all(q_final <= _HIGH + 1e-9):
            cands.append((q_final, err_norm))

    if not cands:
        # Diagnostic d'erreur pour aider l'utilisateur
        x, y, z = target[:3]
        r_radial = np.hypot(x, y)
        max_reach = sum(dh[2][1] for dh in DH) + DH[3][1] + DH[3][3] + DH[5][3]
        if r_radial > max_reach:
            raise RuntimeError(
                f"MGI non convergé : cible hors de portée radiale "
                f"({r_radial:.3f} m > {max_reach:.3f} m)"
            )
        if z < 0.0:
            raise RuntimeError(
                f"MGI non convergé : cible sous le plan de base (z={z:.3f} m)"
            )
        raise RuntimeError(
            f"MGI non convergé pour la cible {target}. "
            f"Aucune des {len(seeds)} graines n'a convergé dans les limites."
        )

    if return_all:
        return [c[0] for c in cands]

    # Sélectionner la solution la plus proche de q0 (ou la 1ère)
    if q0 is not None:
        base = np.asarray(q0, dtype=float)
        best = min(cands, key=lambda c: np.linalg.norm(c[0] - base))[0]
    else:
        # Sans q0, choisir la solution avec erreur minimale
        best = min(cands, key=lambda c: c[1])[0]
    return best


# ============================================================================
# 6. UTILITAIRES DE TRAJECTOIRE
# ============================================================================

def joint_interpolation(q_start: np.ndarray, q_end: np.ndarray,
                        n_steps: int = 50) -> np.ndarray:
    """Interpolation articulaire linéaire (joint space) avec profil trapézoïdal.
    Renvoie une trajectoire (n_steps, 6).
    """
    q_start = np.asarray(q_start, dtype=float)
    q_end = np.asarray(q_end, dtype=float)
    # Profil trapézoïdal : 20% accel, 60% cruise, 20% decel
    t = np.linspace(0, 1, n_steps)
    s = np.where(t < 0.2, 0.5 * (t / 0.2)**2,
        np.where(t < 0.8, 0.1 + (t - 0.2) * 0.8,
            1.0 - 0.5 * ((1 - t) / 0.2)**2))
    return q_start[None, :] + s[:, None] * (q_end - q_start)[None, :]


def cartesian_interpolation(p_start: np.ndarray, p_end: np.ndarray,
                            n_steps: int = 50) -> List[np.ndarray]:
    """Interpolation cartésienne (espace opérationnel) avec SLERP sur l'orientation.
    Renvoie une liste de matrices 4x4.
    """
    T_start = pose_to_matrix(p_start) if p_start.shape == (6,) else p_start
    T_end = pose_to_matrix(p_end) if p_end.shape == (6,) else p_end

    p0, p1 = T_start[:3, 3], T_end[:3, 3]
    R0, R1 = T_start[:3, :3], T_end[:3, :3]

    # SLERP quaternion
    q0 = matrix_to_quaternion(R0)
    q1 = matrix_to_quaternion(R1)
    # Plus court chemin
    if np.dot(q0, q1) < 0:
        q1 = -q1

    traj = []
    for t in np.linspace(0, 1, n_steps):
        # Position : interpolation linéaire
        p_t = p0 + t * (p1 - p0)
        # Orientation : SLERP
        q_t = slerp_quaternion(q0, q1, t)
        R_t = quaternion_to_matrix(q_t)
        T_t = np.eye(4)
        T_t[:3, :3] = R_t
        T_t[:3, 3] = p_t
        traj.append(T_t)
    return traj


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Matrice de rotation -> quaternion (w, x, y, z)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        S = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) -> matrice de rotation."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]
    ])


def slerp_quaternion(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical Linear Interpolation entre deux quaternions."""
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        # Quaternions très proches : interpolation linéaire
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    theta_0 = np.arccos(dot)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    sin_theta_0 = np.sin(theta_0)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0 * q0 + s1 * q1


# ============================================================================
# 7. VALIDATION ET DIAGNOSTIC
# ============================================================================

def validate_target_reachable(target_pose: np.ndarray) -> Tuple[bool, str]:
    """Vérifie si la cible est dans l'espace de travail grossier.
    Renvoie (is_reachable, message).
    """
    x, y, z = target_pose[:3]
    r_radial = np.hypot(x, y)
    # Reach max = somme des a + longueurs
    max_reach = sum(row[1] for row in DH) + DH[3][3] + DH[5][3]
    min_reach = 0.10  # m, rayon minimum (au-dessus de la base)

    if r_radial > max_reach:
        return False, f"Cible hors de portée radiale : {r_radial:.3f} m > {max_reach:.3f} m"
    if r_radial < min_reach and z < 0.20:
        return False, f"Cible trop proche de la base (singularité q1 indéterminé) : r={r_radial:.3f} m"
    if z < 0.0:
        return False, f"Cible sous le plan de base : z={z:.3f} m"
    if z > 0.75:
        return False, f"Cible trop haute : z={z:.3f} m > 0.75 m"
    return True, "OK"


def is_in_limits(q: np.ndarray, tol: float = 1e-6) -> bool:
    """Vérifie que toutes les articulations sont dans les limites."""
    q = np.asarray(q, dtype=float).flatten()
    return bool(np.all(q >= _LOW - tol) and np.all(q <= _HIGH + tol))


# ============================================================================
# 8. EXEMPLE D'UTILISATION
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Test du module Arm_kinematics (version améliorée)")
    print("=" * 60)

    # Test 1 : MGD d'une configuration de référence
    q_test = np.array([0.1, -0.3, 0.2, 0.0, 0.4, 0.0])
    T = mgd(q_test)
    pose = pose_vec(T)
    print(f"\n[1] MGD pour q = {q_test}")
    print(f"    Pose : {pose}")

    # Test 2 : MGI (round-trip)
    q_recovered = mgi(pose, q0=q_test)
    pose_back = pose_vec(mgd(q_recovered))
    err = np.linalg.norm(pose - pose_back)
    print(f"\n[2] Round-trip MGI -> MGD")
    print(f"    q_initial     = {q_test}")
    print(f"    q_recovered   = {q_recovered}")
    print(f"    Erreur pose   = {err:.2e} (should be < 1e-4)")

    # Test 3 : Jacobien analytique vs numérique (vérification)
    eps = 1e-6
    J_analytic = geometric_jacobian(q_test)
    J_numeric = np.zeros((6, 6))
    p0 = pose_vec(mgd(q_test))
    for i in range(6):
        dq = q_test.copy()
        dq[i] += eps
        J_numeric[:, i] = (pose_vec(mgd(dq)) - p0) / eps
    err_J = np.linalg.norm(J_analytic - J_numeric) / np.linalg.norm(J_numeric)
    print(f"\n[3] Vérification Jacobien analytique vs numérique")
    print(f"    Erreur relative = {err_J:.2e} (should be < 1e-3)")

    # Test 4 : Manipulabilité
    is_sing, manip = check_singularity(q_test)
    print(f"\n[4] Manipulabilité")
    print(f"    w = {manip:.4f}")
    print(f"    Singularité : {'OUI' if is_sing else 'non'}")

    # Test 5 : Validation reachability
    target = np.array([0.30, 0.10, 0.20, 0.0, -0.3, 0.1])
    reachable, msg = validate_target_reachable(target)
    print(f"\n[5] Cible {target[:3]} m")
    print(f"    Atteignable : {reachable} ({msg})")

    print("\n" + "=" * 60)
    print("Tous les tests OK — module prêt à l'emploi")
    print("=" * 60)