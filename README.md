# HAJAJI-RAHMANI-ZUMO07

Programme Arduino pour robot **Pololu Zumo 32U4** : suivi de ligne par régulateur PD, maintien de cap au gyroscope, contournement d'obstacle avec les capteurs de proximité, puis parking final avec chronométrage du parcours. Le robot se pilote par liaison série (`Serial1`, 38400 bauds, par exemple via un module Bluetooth).

## Matériel

- Zumo 32U4 avec carte de 5 capteurs de ligne (les 2 cavaliers de la carte capteurs doivent être en position **intérieure**)
- Module série sur `Serial1` (optionnel, pour les commandes)

## Installation

1. Installer l'IDE Arduino et la bibliothèque **Zumo32U4** (gestionnaire de bibliothèques).
2. Sélectionner la carte *Pololu A-Star 32U4*.
3. Ouvrir `HAJAJI-RAHMANI-ZUMO07.ino` (le dossier doit garder le même nom que le fichier `.ino`) et téléverser.

Au démarrage, appuyer sur le bouton **B** : le robot calibre ses capteurs de ligne en pivotant, puis joue une mélodie quand il est prêt.

## Commandes série

| Touche | Action |
|---|---|
| `g` | Lancer le parcours |
| `s` | Arrêter le robot (affiche le temps écoulé) |
| `c` | Recalibrer les capteurs de ligne |
| `r` | Relancer après un arrêt |
| `t` | Afficher le chrono en cours |
| `d` | Afficher les distances gauche/droite des encodeurs |
| `x` | Afficher la tension batterie (mV) |

## Structure

```
HAJAJI-RAHMANI-ZUMO07.ino   # logique principale (PID, machine d'états, commandes série)
gyro_use.h / gyro_use.cpp   # configuration et lecture du gyroscope (angle de rotation sur Z)
```

## Crédits

Les fonctions gyroscope reprennent l'exemple *TurnSensor* fourni par Pololu avec la bibliothèque Zumo32U4.
