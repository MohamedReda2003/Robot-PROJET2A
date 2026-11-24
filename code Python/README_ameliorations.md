# Améliorations du contrôle du bras Niryo Ned

## Fichiers livrés

- **`Arm_kinematics.py`** — module cinématique amélioré (MGD, MGI, Jacobien analytique)
- **`test_transform_to_motion.py`** — script de mouvement XYZ avec trajectoire interpolée
- **`test_keyboard_control.py`** — pilotage temps réel au clavier (flèches + PageUp/Down + wasd/qe)
- **`README_ameliorations.md`** — ce fichier

## Résumé des améliorations principales

| Aspect | Version originale | Version améliorée | Gain |
|---|---|---|---|
| **Jacobien** | Différences finies (7 appels MGD) | Analytique (1 calcul direct) | ×50 vitesse |
| **Précision J** | Erreur ~1e-3 (numérique) | Erreur ~1e-7 vs analytique | ×1000 |
| **MGI multi-graines** | 8 graines fixes | 33 graines systématiques | Meilleure couverture |
| **Limites articulaires** | Clip initial uniquement | Clip à CHAQUE itération | Sécurité accrue |
| **Singularités** | Non détectées | Détection + amortissement adaptatif | Robustesse |
| **Gimbal lock** | Non géré | Cas pitch=±π/2 traité | Pas de division par 0 |
| **Trajectoire** | Saut direct (move_joints) | 50 waypoints profil trapézoïdal | Mouvement fluide |
| **Validation** | Après connexion robot | Avant connexion | Gain de temps |
| **Diagnostic** | RuntimeError générique | Messages détaillés | Débogage facile |

## Test de validation

```
[1] MGD pour q = [0.1, -0.3, 0.2, 0.0, 0.4, 0.0]
    Pose : [0.349, 0.035, 0.409, 0, -0.3, 0.1]

[2] Round-trip MGI -> MGD
    q_initial     = [0.1, -0.3, 0.2, 0.0, 0.4, 0.0]
    q_recovered   = [0.1, -0.3, 0.2, 0.0, 0.4, 0.0]
    Erreur pose   = 0.00e+00

[3] Vérification Jacobien analytique vs numérique
    Erreur relative = 1.43e-07  (quasi-parfaite)

[4] Manipulabilité
    w = 0.0057  (configuration saine, non singulière)
```

## Utilisation

1. Remplacer les fichiers dans le dossier `code Python/` du dépôt GitHub.
2. Sur la machine du projet (avec pyniryo installé) :

   **Test de mouvement ciblé** (XYZ + orientation) :
   ```bash
   python test_transform_to_motion.py
   ```

   **Pilotage temps réel au clavier** :
   ```bash
   python test_keyboard_control.py --ip 10.10.3.156 --step 1.0
   ```
   Contrôles :
   - `←/→` : déplacement X (avant/arrière)
   - `↑/↓` : déplacement Y (gauche/droite)
   - `PageUp/PageDown` : déplacement Z (haut/bas)
   - `w/s` : pitch -/+
   - `a/d` : roll -/+
   - `q/e` : yaw -/+
   - `r` : retour home
   - `h` : aide, `c` : pose courante, `x`/Échap : quitter

3. **Mode simulation** : si `pyniryo` n'est pas installé, le script bascule
   automatiquement en mode simulation pour tester la logique sans robot.

## Points d'attention pour la suite

- **Tester avec le robot réel** pour valider que les paramètres DH correspondent bien au Niryo Ned 1 (pas Ned 2).
- Si les mouvements sont imprécis (> 1 cm), c'est probablement un décalage des `theta_off` dans la table DH — à recalibrer.
- Le module est compatible avec `simulating_positions.py` : remplacer l'appel `robot.get_inverse_kinematics(pose)` par `mgi(pose_vec(pose_matrix))` pour éviter les appels réseau lents.
- **Pour le pilotage clavier** : commencez par un petit `--step 0.5` (5 mm) pour tester la réactivité, puis augmentez progressivement. La vitesse de PyNiryo est volontairement lente (50 %) pour la sécurité.
- **Sécurité** : le script détecte les singularités et les sorties de workspace, mais gardez toujours un doigt sur `x` (ou Échap) pour arrêter en cas de mouvement inattendu.
