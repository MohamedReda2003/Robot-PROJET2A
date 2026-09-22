# Projet — Robot de désherbage autonome (UGV + bras manipulateur)

Ce document détaille les axes de travail du projet, avec pour chacun les objectifs, les étapes concrètes, les livrables attendus et les outils recommandés.

---

## Sommaire

1. [Vision et détection (CV)](#1-vision-et-détection-cv)
2. [Cinématique du bras manipulateur (MGD/MGI)](#2-cinématique-du-bras-manipulateur-mgdmgi)
3. [Châssis et plateforme mécanique (UGV)](#3-châssis-et-plateforme-mécanique-ugv)
4. [Motorisation et asservissement bas niveau](#4-motorisation-et-asservissement-bas-niveau)
5. [Localisation et navigation autonome](#5-localisation-et-navigation-autonome)
6. [Interface de gestion des zones](#6-interface-de-gestion-des-zones)
7. [Intégration et tests système](#7-intégration-et-tests-système)

---

## 1. Vision et détection (CV)

**Objectif** : améliorer/adapter le système de détection des mauvaises herbes hérité de l'an dernier pour qu'il fonctionne en conditions réelles depuis l'UGV en mouvement, et qu'il fournisse une position exploitable par le bras.

### Étapes

- **1.1 Audit de l'existant**
  Récupérer le code et le modèle de l'an dernier. Identifier : type d'approche (classique ou DL), matériel cible, taux de détection, temps d'inférence, limites connues (luminosité, occlusion, angle de caméra).

- **1.2 Choix/consolidation de l'approche**
  Décider si l'on garde l'approche existante ou si on la fait évoluer (ex : passer d'un filtrage colorimétrique à un YOLO léger, ou l'inverse pour gagner en vitesse). Documenter le choix et sa justification.

- **1.3 Constitution/enrichissement du dataset**
  Si DL : collecter des images supplémentaires dans les conditions réelles du terrain cible (luminosité, angle caméra montée sur UGV). Annoter (LabelImg, Roboflow, CVAT). Augmenter les données (rotation, luminosité, bruit).

- **1.4 Entraînement / calibration**
  Entraîner ou recalibrer le modèle. Mesurer précision, rappel, temps d'inférence sur le matériel embarqué cible (Raspberry Pi / Jetson).

- **1.5 Transformation image → coordonnées monde**
  Convertir la position pixel de la mauvaise herbe détectée en coordonnées réelles (repère caméra puis repère robot), via calibration caméra (matrice intrinsèque/extrinsèque, homographie si caméra fixe au-dessus du sol).

- **1.6 Interface avec le bras**
  Définir le format de message envoyé au module de commande du bras (ex : coordonnées x, y, z dans le repère base du bras).

### Livrables
- Rapport d'audit du système existant
- Dataset annoté (si DL)
- Modèle entraîné + script d'inférence
- Module de conversion pixel → coordonnées robot
- Documentation de l'interface CV → bras

### Outils suggérés
OpenCV, Python, PyTorch/Ultralytics (YOLOv8), Roboflow/CVAT pour l'annotation, calibration caméra OpenCV (`cv2.calibrateCamera`).

---

## 2. Cinématique du bras manipulateur (MGD/MGI)

**Objectif** : documenter et valider les modèles géométriques du bras (déjà construit et fonctionnel) pour permettre son pilotage à partir d'une position cible fournie par la vision, et son intégration logicielle avec l'UGV.

### Étapes

- **2.1 Relevé des paramètres géométriques**
  Mesurer/récupérer les longueurs de segments, angles limites de chaque articulation, nombre de degrés de liberté (DDL). Construire le tableau des paramètres de Denavit-Hartenberg (DH).

- **2.2 Modèle Géométrique Direct (MGD)**
  Calculer la matrice de transformation homogène de la base vers l'effecteur en fonction des angles articulaires (produit des matrices DH). Vérifier par simulation (MATLAB/Python + Robotics Toolbox) que la position calculée correspond aux mesures physiques du bras réel.

- **2.3 Modèle Géométrique Inverse (MGI)**
  À partir d'une position/orientation cible de l'effecteur, calculer les angles articulaires nécessaires. Traiter les cas de multiplicité de solutions et les singularités. Valider par comparaison avec des positions test mesurées sur le bras réel.

- **2.4 Vérification des limites articulaires et zones atteignables**
  Définir l'espace de travail (workspace) réel du bras à partir des limites mécaniques. Vérifier que les zones où les mauvaises herbes peuvent apparaître sont bien atteignables compte tenu du positionnement du bras sur l'UGV.

- **2.5 Intégration logicielle**
  Encapsuler MGD/MGI dans un module réutilisable (fonction ou service) prenant en entrée une position cible (issue de la vision) et retournant les consignes articulaires à envoyer aux moteurs du bras.

### Livrables
- Tableau des paramètres DH
- Script/fonction MGD validé
- Script/fonction MGI validé avec gestion des cas limites
- Schéma de l'espace de travail atteignable
- Module d'intégration CV → MGI → commande moteurs

### Outils suggérés
Python (NumPy, `roboticstoolbox-python`), MATLAB (Robotics System Toolbox) si disponible, simulation avant test sur le bras réel.

---

## 3. Châssis et plateforme mécanique (UGV)

**Objectif** : disposer d'une base mobile capable de porter la caméra, le bras et l'électronique, adaptée au terrain de la parcelle.

### Étapes

- **3.1 Cahier des charges mécanique**
  Charge utile à transporter (bras + électronique + batteries), type de terrain (plat, irrégulier, inter-rangs de culture), vitesse cible, garde au sol nécessaire.

- **3.2 Choix de la configuration**
  Roues (4 roues motrices, différentiel) vs chenilles, dimensionnement des moteurs en fonction du poids total et de la pente à franchir.

- **3.3 Conception/adaptation du châssis**
  Modélisation CAO (SolidWorks/Fusion360) si fabrication, ou sélection d'une plateforme existante à adapter. Prévoir les points de fixation pour caméra, bras et boîtier électronique.

- **3.4 Fabrication/assemblage**
  Usinage, impression 3D des supports, assemblage mécanique.

- **3.5 Validation mécanique**
  Test de charge, test de stabilité (surtout lors des mouvements du bras en extension), test sur terrain réel ou simulé.

### Livrables
- Cahier des charges mécanique
- Fichiers CAO du châssis
- Châssis assemblé
- Rapport de validation (charge, stabilité)

---

## 4. Motorisation et asservissement bas niveau

**Objectif** : permettre à l'UGV de se déplacer de manière contrôlée et précise (vitesse, direction).

### Étapes

- **4.1 Choix des actionneurs**
  Moteurs + drivers adaptés au couple/vitesse requis, encodeurs pour le retour de position/vitesse.

- **4.2 Câblage et intégration électronique**
  Connexion moteurs/drivers/encodeurs à la carte de contrôle (Arduino/STM32/Raspberry Pi + carte moteur).

- **4.3 Commande bas niveau**
  Développement du firmware de commande des moteurs (PWM, sens de rotation).

- **4.4 Asservissement**
  Implémentation d'un régulateur PID pour la vitesse de chaque roue/moteur, puis pour la trajectoire globale (différentiel gauche/droite).

- **4.5 Tests**
  Test de déplacement en ligne droite, en rotation, réponse à une consigne de vitesse variable.

### Livrables
- Schéma électronique de motorisation
- Firmware de commande bas niveau
- PID réglé et validé
- Rapport de tests de déplacement

---

## 5. Localisation et navigation autonome

**Objectif** : permettre à l'UGV de connaître sa position et de parcourir seul une parcelle définie en évitant les obstacles.

### Étapes

- **5.1 Choix des capteurs de localisation**
  Odométrie (encodeurs), IMU (orientation), GPS/RTK-GPS si précision centimétrique nécessaire.

- **5.2 Fusion de capteurs**
  Filtre de Kalman étendu (EKF) pour fusionner odométrie + IMU (+ GPS) et obtenir une position fiable malgré le bruit.

- **5.3 Détection d'obstacles**
  Choix de capteurs (LiDAR, ultrasons, caméra de profondeur) et algorithme de détection d'obstacles en temps réel.

- **5.4 Planification de trajectoire**
  Définition de la parcelle (points GPS ou repères), génération d'un parcours couvrant la zone (ex : balayage en lignes), algorithme d'évitement d'obstacles dynamique (ex : Dynamic Window Approach, VFH).

- **5.5 Intégration ROS (recommandé)**
  Structurer la navigation via ROS/ROS2 (packages `robot_localization`, `move_base`/`nav2`) pour bénéficier d'outils existants plutôt que tout recoder.

- **5.6 Tests terrain**
  Validation du suivi de trajectoire, de l'évitement d'obstacles, de la couverture complète de la parcelle test.

### Livrables
- Architecture de fusion de capteurs (EKF)
- Module de détection d'obstacles
- Algorithme de planification de trajectoire
- Rapport de tests de navigation autonome

### Outils suggérés
ROS2, Python/C++, `robot_localization`, `nav2`, simulation Gazebo avant test réel.

---

## 6. Interface de gestion des zones

**Objectif** : permettre à un utilisateur de définir/visualiser les zones de la parcelle et de suivre l'activité du robot.

### Étapes

- **6.1 Spécification fonctionnelle**
  Définir les besoins : tracer/éditer les limites de la parcelle et des sous-zones, lancer une mission, visualiser en temps réel la position du robot et les mauvaises herbes traitées.

- **6.2 Choix technique**
  Application web (carte interactive type Leaflet/Mapbox) ou application locale connectée à l'UGV en WiFi.

- **6.3 Développement backend**
  Communication avec l'UGV (MQTT, ROS bridge, ou API REST) pour envoyer les zones et recevoir les données de position/état.

- **6.4 Développement frontend**
  Interface carte, formulaire de définition de zones, tableau de bord d'état (batterie, position, mauvaises herbes traitées).

- **6.5 Tests d'intégration**
  Vérifier la synchronisation temps réel entre interface et robot.

### Livrables
- Spécification fonctionnelle
- Interface fonctionnelle (web ou desktop)
- Documentation de communication interface ↔ UGV

---

## 7. Intégration et tests système

**Objectif** : assembler tous les sous-systèmes en un robot fonctionnel et valider le comportement global.

### Étapes

- **7.1 Intégration matérielle**
  Montage du bras, de la caméra et de l'électronique sur le châssis final.

- **7.2 Intégration logicielle**
  Connexion des modules : navigation → détection CV → MGI → commande bras, le tout piloté depuis l'interface.

- **7.3 Tests unitaires par module**
  Valider chaque brique séparément avant le test global (déplacement seul, détection seule, arrachage seul).

- **7.4 Tests système en conditions réelles**
  Parcours complet d'une parcelle test avec détection et arrachage effectifs, en présence de cultures factices ou réelles.

- **7.5 Débogage et optimisation**
  Ajustement des paramètres (vitesse, seuils de détection, précision du bras) suite aux tests.

- **7.6 Documentation finale**
  Rédaction du rapport global et préparation de la soutenance/démonstration.

### Livrables
- Robot assemblé et fonctionnel
- Rapport de tests système
- Documentation finale et support de présentation

---

## Dépendances entre axes

- La **vision (1)** doit fournir un format de position exploitable avant que l'intégration avec le **bras (2)** soit possible.
- Le **châssis (3)** doit être disponible avant les tests réels de **motorisation (4)** et de **navigation (5)**.
- La **navigation (5)** et l'**interface (6)** peuvent être développées en parallèle une fois la communication UGV définie.
- L'**intégration (7)** ne peut démarrer complètement que lorsque les axes 1 à 6 ont chacun un livrable testé indépendamment.
