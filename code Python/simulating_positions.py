import math
import numpy as np
import matplotlib.pyplot as plt
from pyniryo import NiryoRobot, PoseObject

def generate_workspace(robot_ip, step_cm=0.001):
    """
    Balaye la zone devant le robot et renvoie la liste des points (x, y, z) valides.
    step_cm : Pas de recherche en cm (ex: 5 cm pour un calcul rapide, 2 cm pour plus de précision).
    """
    robot = NiryoRobot(robot_ip)
    
    # Plages de recherche (en mètres)
    step = step_cm / 100.0
    x_range = np.arange(0.10, 0.38, step)
    y_range = np.arange(-0.25, 0.25, step)
    z_range = np.arange(0.05, 0.35, step)

    valid_points = []

    print(f"🔍 Calcul de l'espace de travail en cours (pas de {step_cm} cm)...")

    for x in x_range:
        for y in y_range:
            for z in z_range:
                # Calcul automatique du Yaw aligné vers la cible pour maximiser le MGI
                yaw = math.atan2(y, x)
                pose = PoseObject(x=x, y=y, z=z, roll=0.0, pitch=1.57, yaw=yaw)

                try:
                    # Si le MGI réussit, la position est valide
                    robot.get_inverse_kinematics(pose)
                    valid_points.append((round(x, 3), round(y, 3), round(z, 3)))
                except:
                    continue

    robot.close_connection()
    print(f"✅ {len(valid_points)} points valides trouvés !")
    return valid_points


def plot_workspace(valid_points):
    """
    Affiche l'espace de points 3D atteignables.
    """
    if not valid_points:
        print("Aucun point valide à afficher.")
        return

    pts = np.array(valid_points)
    
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Tracé des points accessibles
    img = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2], cmap='viridis', s=20, alpha=0.8)
    
    # Repère de la base du Ned 1
    ax.scatter(0, 0, 0, color='red', s=100, label='Base Ned 1 (0,0,0)')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("Espace de points accessibles pour Niryo Ned 1")
    fig.colorbar(img, ax=ax, label='Hauteur Z (m)')
    ax.legend()
    plt.show()


# --- EXÉCUTION ---
if __name__ == "__main__":
    ROBOT_IP = "10.10.3.156"  # Mets l'IP de ton robot
    
    # 1. Génération de la liste des points accessibles
    points_accessibles = generate_workspace(ROBOT_IP, step_cm=0.1)

    # 2. Affichage 3D interactif
    plot_workspace(points_accessibles)

    # 3. Exemple : Filtrer les points à une hauteur Z donnée (ex: plan de travail à Z = 15 cm)
    points_table = [p for p in points_accessibles if p[2] == 0.13]
    print(f"\nExemples de points disponibles à Z = 0.13 m :")
    for p in points_table[:5]:  # Affiche les 5 premiers
        print(f" -> x={p[0]}, y={p[1]}, z={p[2]}")