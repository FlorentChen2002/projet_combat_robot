# PatientHunter — Simovies

Projet réalisé dans le cadre de l'UE **CPA — Algorithmique d'essaims** à Sorbonne Université (2025–2026).

**Équipe :** Chen · Fallavier · Tang  
**Encadrant :** Binh-Minh Bui-Xuan

---

## Description

PatientHunter est une implémentation de robots autonomes pour le simulateur **Simovies**.  
Chaque équipe aligne cinq robots répartis en deux rôles :

| Rôle | Classe | Caractéristiques |
|------|--------|-----------------|
| Tireur (×3) | `TeamAMainBotChenFallavierTang` | Résistant, lent, radar court, peut tirer |
| Éclaireur (×2) | `TeamASecondaryBotChenFallavierTang` | Rapide, fragile, radar long, ne peut pas tirer |

L'éclaireur détecte et transmet les positions ennemies par broadcast. Le tireur exploite ces informations pour viser des cibles qu'il n'aurait jamais vues seul.

---

## Structure des fichiers

```
algorithms/
├── PatientHunterBase.java                  # Classe abstraite partagée (cerveau commun)
├── TeamAMainBotChenFallavierTang.java      # Entrée équipe A — tireur
├── TeamASecondaryBotChenFallavierTang.java # Entrée équipe A — éclaireur
├── TeamBMainBotChenFallavierTang.java      # Entrée équipe B — tireur
└── TeamBSecondaryBotChenFallavierTang.java # Entrée équipe B — éclaireur
```

---

## Fonctionnement global

### Ce que fait chaque robot à chaque tour

1. Mise à jour de la position (odométrie)
2. Lecture des messages des alliés
3. Lecture du radar
4. Mise à jour de la mémoire partagée (ennemis, projectiles, positions alliées)
5. Prise de décision (tir, déplacement, déblocage)

### Communication entre robots

Les robots s'échangent des messages broadcast au format :

```
PHR|<TYPE>|<X>|<Y>|<DIR_OU_NA>|<ID_EMETTEUR>
```

| Type | Signification |
|------|--------------|
| `ENEMY` | Ennemi détecté directement |
| `ENEMY_PREDICTED` | Ennemi inféré depuis une trajectoire de balle |
| `BULLET` | Balle ennemie repérée |
| `FIRE` | Tir effectué (pour filtrage anti-fratricide) |
| `POS` | Position propre diffusée |

### Stratégie du tireur (Main)

- **Départ :** formation en éventail sur 380 tours pour couvrir l'axe de progression
- **Tir :** seulement si la cible a été vue il y a moins de 6 tours, hors portée des alliés
- **Anti-fratricide :** triple garde (capteur frontal + liste broadcast + radar)

### Stratégie de l'éclaireur (Secondary)

- **Phase 1 (< 4000 tours) :** avance en ligne droite, recule si un ennemi est à moins de 480 unités
- **Phase 2 (≥ 4000 tours) :** orbite autour de l'ennemi à distance idéale de 600 unités
- **Fuite :** si les PV passent sous 20%, fuit à l'opposé de l'ennemi le plus proche

---

## Installation et lancement

1. Télécharger et lancer le simulateur **Simovies**
2. Copier les fichiers `.java` dans le dossier `algorithms/` du simulateur
3. Compiler le projet
4. Charger les quatre classes dans le simulateur et lancer une partie

---

## Résultats

| Adversaire | Pertes | Éliminations |
|------------|--------|-------------|
| RandomFire | 1–2/5 | 4–5/5 |
| Berzerk | 1–4/5 | 3–5/5 |
| Fugitive | 2–3/5 | 2–5/5 |
| CampFire/CampBot | 4–5/5 | 2–3/5 |
| Hunter vs Hunter | 3–4/5 | 2–4/5 |

---

## Rapport

Le rapport technique (SMA vs. Actor Model) est disponible dans `rapport_patient_hunter.pdf`.  
Le source LaTeX est dans `rapport_patient_hunter.tex`.
