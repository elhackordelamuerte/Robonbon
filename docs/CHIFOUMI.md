# 🎮 Pierre - Feuille - Ciseaux (Chifoumi) avec le bras SO-ARM101

Ce document explique le fonctionnement complet du projet **Pierre - Feuille - Ciseaux** contre le bras robotisé SO-ARM101 en simulation (MuJoCo / RViz), couplé à une caméra réelle (webcam) avec vision par ordinateur (OpenCV + MediaPipe).

---

## 1. Comment l'expliquer à des personnes sans bagage en robotique ? (Vulgarisation)

Si vous devez présenter ce projet à des personnes extérieures à la robotique ou à l'informatique industrielle, voici une explication simple en 4 grands principes :

### 👁️ 1. « Les yeux » : La vision par ordinateur
> *« Comment l'ordinateur comprend-il ce que fait notre main ? »*
>
> Une webcam filme généralement 30 images par seconde. Chaque image n'est pour l'ordinateur qu'une grande grille de points colorés (pixels).  
> Grâce à l'intelligence artificielle (**MediaPipe**, développé par Google), le système repère instantanément **21 points clés** sur votre main : le poignet, la base de chaque doigt et chaque articulation jusqu'au bout des ongles.  
> C'est l'équivalent d'un squelette virtuel. Ensuite, un algorithme géométrique très simple mesure la distance entre le bout de chaque doigt et la paume :
> - **Tous les doigts repliés vers la paume** ➔ **Pierre 🪨**
> - **Tous les doigts tendus et écartés** ➔ **Feuille 📄**
> - **Seuls l'index et le majeur dressés (en V)** ➔ **Ciseaux ✂️**

---

### 🦾 2. « Les muscles » : Les servomoteurs et les articulations
> *« Comment le bras robotisé bouge-t-il ? »*
>
> Le bras SO-ARM101 est inspiré d'un bras humain à échelle réduite. Il possède **6 moteurs intelligents** (appelés servomoteurs) :
> 1. La base qui pivote de gauche à droite (l'épaule horizontale).
> 2. L'épaule qui lève ou baisse le bras.
> 3. Le coude qui plie l'avant-bras.
> 4. Le poignet vertical qui incline la main.
> 5. Le poignet rotatif qui tourne la main sur elle-même (comme pour tourner une poignée de porte).
> 6. La pince (le préhenseur) qui s'ouvre et se ferme de 0 % à 100 %.
>
> Pour jouer :
> - **Pierre** : le bras ferme sa pince à 0 % pour former un poing solide face à vous.
> - **Feuille** : le bras tourne son poignet à plat et ouvre sa pince à 100 % comme une paume ouverte.
> - **Ciseaux** : le bras incline sa pince entrouverte à 45 % pour imiter deux lames de ciseaux prêtes à couper.

---

### ⏱️ 3. « Le rythme » : La synchronisation temps réel
> *« Comment s'assure-t-on que le robot ne triche pas (ou que l'humain ne triche pas) ? »*
>
> Au Chifoumi, les deux joueurs doivent abattre leur coup **exactement en même temps**.  
> Le système joue le rôle d'arbitre et de chef d'orchestre :
> 1. Le robot effectue 3 mouvements de balancement de haut en bas en scandant :  
>    *« 1... Pierre ! » ➔ « 2... Feuille ! » ➔ « 3... Ciseaux ! »*
> 2. Au mot **« SHOOT ! »**, le robot dévoile son coup tiré au sort et le programme fige précisément l'image de votre main à cette milliseconde précise.  
> Il est donc impossible pour le robot d'attendre de voir votre main pour choisir son coup, et impossible pour le joueur de changer son geste en retard !

---

### ⚖️ 4. « L'arbitre » : La matrice de décision
> L'arbitre compare les deux coups selon la règle universelle :
> - La **Pierre** écrase les **Ciseaux**.
> - Les **Ciseaux** coupent la **Feuille**.
> - La **Feuille** enveloppe la **Pierre**.
> - Deux coups identiques donnent une **Égalité**.  
>
> En cas de victoire du robot, il effectue une petite **danse de la victoire** en agitant la pince. S'il perd, il **baisse la tête de dépit** !

---

## 2. Architecture Technique

```
┌────────────────────────────────────────────────────────────────────────┐
│                          HÔTE macOS / PC                               │
│                                                                        │
│   Webcam réelle                                                        │
│         │                                                              │
│         ▼                                                              │
│  [ hand_detector.py ]                                                  │
│   - OpenCV (AVFoundation)                                              │
│   - MediaPipe Hands (21 landmarks)                                     │
│   - Classification & lissage                                           │
│         │ (Geste: PIERRE / FEUILLE / CISEAUX)                          │
│         ▼                                                              │
│  [ chifoumi_game.py ] ◄── Clavier (Espace, A, R)                       │
│   - Machine d'états (Attente, Compte 1-2-3, Verdict)                   │
│   - HUD graphique animé (scores, compte, bandeaux)                     │
│   - Arbitre temps réel                                                 │
│         │                                                              │
│         │ Requêtes HTTP POST /api/commande ou /api/lecture             │
└─────────┼──────────────────────────────────────────────────────────────┘
          │ (Port 8080 - tek5-panel)
          ▼
┌────────────────────────────────────────────────────────────────────────┐
│                        CONTENEURS DOCKER                               │
│                                                                        │
│   Conteneur 'control' (ROS 2 Jazzy) :                                  │
│     - joint_panel (panel.py) : API REST -> /joint_command              │
│     - driver_node (SO101Sim MuJoCo)                                    │
│     - robot_state_publisher -> /tf                                     │
│                                                                        │
│   Conteneur 'tek5-rviz' (noVNC port 6080) :                            │
│     - RViz2 affiché dans le navigateur (rendu 3D temps réel du bras)    │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Détail des Poses Robot (SO-ARM101)

Les consignes sont envoyées sur le topic ROS 2 `/joint_command` (`sensor_msgs/JointState`).  
Angles en **radians** pour les 5 axes, ouverture de pince en **%** (0 = fermée, 100 = ouverte) :

| Articulation | READY (Garde) | PUMP_UP (Haut) | PUMP_DOWN (Bas) | PIERRE 🪨 | FEUILLE 📄 | CISEAUX ✂️ |
|---|---|---|---|---|---|---|
| `shoulder_pan` | 0.00 rad | 0.00 rad | 0.00 rad | 0.00 rad | 0.00 rad | 0.00 rad |
| `shoulder_lift` | -0.50 rad | -0.75 rad | -0.35 rad | -0.42 rad | -0.45 rad | -0.40 rad |
| `elbow_flex` | 0.70 rad | 0.55 rad | 0.85 rad | 0.75 rad | 0.65 rad | 0.70 rad |
| `wrist_flex` | 0.40 rad | 0.65 rad | 0.20 rad | 0.35 rad | 0.10 rad | 0.25 rad |
| `wrist_roll` | 0.00 rad | 0.00 rad | 0.00 rad | 0.00 rad | **1.57 rad** | **1.57 rad** |
| `gripper` | 15.0 % | 10.0 % | 10.0 % | **0.0 %** | **100.0 %** | **45.0 %** |

---

## 4. Guide d'Utilisation Pas-à-Pas

### Étape 1 : Lancer la stack simulation & RViz
Dans un premier terminal :
```bash
./scripts/start.sh
```
- Le panneau web s'ouvre sur : `http://localhost:8080`
- RViz s'ouvre dans votre navigateur sur : `http://localhost:6080`

### Étape 2 : Lancer le jeu Chifoumi
Dans un second terminal :
```bash
./scripts/chifoumi.sh                 # Maintien de la pose par défaut (6 secondes)
./scripts/chifoumi.sh --hold 8        # Maintient la pose pendant 8 secondes dans RViz
```
*(Ou directement avec uv : `uv run handGesture/chifoumi_game.py --hold 8`)*

### Étape 3 : Commandes en jeu
| Touche | Action |
|---|---|
| **`ESPACE`** | Lancer une manche (décompte 1, 2, 3... Chifoumi !) |
| **`A`** | Activer / désactiver le mode automatique (enchaîne les manches en continu) |
| **`R`** | Réinitialiser les scores |
| **`C`** | Replacer le robot en garde |
| **`Q`** ou **`Échap`** | Quitter le jeu |

---

## 5. Test Isolé de la Détection Vidéo

Pour tester uniquement la détection de la main et vérifier que votre webcam reconnaît bien vos doigts sans lancer la simulation :
```bash
./scripts/chifoumi.sh --test
```
*(Ou : `uv run handGesture/hand_detector.py`)*
