import numpy as np

# --- Table DH modifiée (alpha_{i-1}, a_{i-1}, theta_off_i, d_i) ---
DH = [
    (0.0,             0.0,    0.0,               0.1715),
    (np.pi/2,         0.0,    np.radians(86.892),0.0),
    (0.0,             0.221326, np.radians(3.108),0.0),
    (np.pi/2,         0.0325, 0.0,               0.235),
    (-np.pi/2,        0.0,    0.0,               0.0),
    (np.pi/2,         0.00925,np.pi/2,           0.0412),
]

# Transform du repère 6 -> repère outil réel (fixe)
T6_TOOL = np.array([
    [ 0,-1, 0, 0],
    [ 0, 0,-1, 0],
    [ 1, 0, 0, 0],
    [ 0, 0, 0, 1]], dtype=float)

JOINT_LIMITS = [(-2.967,2.967),(-2.094,0.611),(-1.344,1.571),
                (-2.094,2.094),(-1.745,0.960),(-2.531,2.531)]

def dh_transform(alpha, a, theta, d):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,      -st,      0,     a],
        [st*ca,  ct*ca,   -sa, -sa*d],
        [st*sa,  ct*sa,    ca,  ca*d],
        [0,0,0,1]])

def mgd(q):
    """MGD : q (rad, 6) -> T (4x4) pose de l'outil dans le repère base."""
    T = np.eye(4)
    for i, (alpha, a, off, d) in enumerate(DH):
        T = T @ dh_transform(alpha, a, q[i] + off, d)
    return T @ T6_TOOL

def pose_vec(T):
    """4x4 -> [x,y,z, roll,pitch,yaw] (rad)."""
    p = T[:3,3]
    R = T[:3,:3]
    pitch = np.arctan2(-R[2,0], np.hypot(R[0,0],R[1,0]))
    roll  = np.arctan2(R[2,1], R[2,2])
    yaw   = np.arctan2(R[1,0], R[0,0])
    return np.concatenate([p,[roll,pitch,yaw]])

def geometric_jacobian(q, eps=1e-6):
    """Jacobien 6x6 par différences finies (robuste, pas besoin de dériver
    analytiquement chaque colonne)."""
    J = np.zeros((6,6))
    p0 = pose_vec(mgd(q))
    for i in range(6):
        dq = np.array(q, dtype=float)
        dq[i] += eps
        J[:,i] = (pose_vec(mgd(dq)) - p0) / eps
    return J
def _wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def _lm_step(target, q, max_iter=80):
    """Une passe Levenberg-Marquardt à amortissement adaptatif, sans clip dur."""
    q = np.clip(np.array(q, dtype=float), [l[0] for l in JOINT_LIMITS], [l[1] for l in JOINT_LIMITS])
    lam = 1e-3
    for _ in range(max_iter):
        err = target - pose_vec(mgd(q))
        err[3:] = _wrap_pi(err[3:])
        if np.linalg.norm(err[:3]) < 1e-4 and np.linalg.norm(err[3:]) < 1e-3:
            return q
        J = geometric_jacobian(q)
        A = J.T @ J + lam * np.diag(np.diag(J.T @ J) + 1e-12)
        try:
            dq = np.linalg.solve(A, J.T @ err)
        except np.linalg.LinAlgError:
            return q
        q_new = q + dq
        err_new = target - pose_vec(mgd(q_new))
        err_new[3:] = _wrap_pi(err_new[3:])
        if err_new @ err_new < err @ err:
            q, lam = q_new, max(lam / 3, 1e-9)
        else:
            lam = min(lam * 5, 1e6)
    return q


#def mgi(target_pose, q0=None, max_iter=200, tol=1e-6, damping=1e-3):
def mgi(target_pose, q0=None):
    """MGI multi-graines : essaie plusieurs branches, garde les solutions
    dans les limites, retourne la plus proche des articulations courantes."""
    target = np.array(target_pose, dtype=float)
    low = np.array([l[0] for l in JOINT_LIMITS])
    high = np.array([l[1] for l in JOINT_LIMITS])
    seeds = [np.asarray(q0, float)] if q0 is not None else []
    q1 = np.arctan2(target[1], target[0])
    for q2, q3 in ((-1.2, -0.8), (-0.5, 0.5), (0.3, 1.2), (0.1, 0.6)):
        seeds.append(np.array([q1, q2, q3, 0.0, -0.3, 0.0]))
        seeds.append(np.array([q1, q2, q3, 0.0, 0.5, 0.0]))
    cands = []
    for s in seeds:
        q = _lm_step(target, s)
        e = target - pose_vec(mgd(q))
        e[3:] = _wrap_pi(e[3:])
        if (np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3
                and np.all(q >= low) and np.all(q <= high)):
            cands.append(q)
    if not cands:
        raise RuntimeError("MGI non convergé")
    base = np.asarray(q0, float) if q0 is not None else cands[0]
    return min(cands, key=lambda q: np.linalg.norm(q - base))

# --- Exemple d'utilisation avec le robot réel (PyNiryo) ---
if __name__ == "__main__":
    q_test = [0.1, -0.3, 0.2, 0.0, 0.4, 0.0]
    T = mgd(q_test)
    print("Pose calculée :", pose_vec(T))
