# Simulateur actuel

Tu simules déjà :

- résistance aérodynamique

- résistance au roulement

- résistance de pente

- moteur limité en couple et puissance

- transmission

- freinage

- cinématique correcte

- puissance aux roues

- consommation carburant cumulée

- affichage graphique

 # Prochaines extensions

## Dynamique véhicule :

- courbes et direction

- limites d’adhérence

- inertie de rotation roues + moteur

- vent, pluie, variation densité air

- température moteur

- gestion carburant ou batterie

## Chaîne de transmission :

- boîte de vitesses réelle

- embrayage

- glissement transmission

- différentiel

- pertes mécaniques internes

- Moteur :

## BSFC variable selon couple et régime

- rendement volumétrique

- temps de réponse moteur

- turbo lag si moteur suralimenté

- carte couple complète 3D (charge × régime)

## Freinage :

- freinage hydraulique réel

- montée en pression

- fading

- ABS basique

## Parcours :

- profil altimétrique complexe

- virages avec rayon

- revêtement (adhérence variable)


## Structuration du projet 
Revoir la structuration avant de continuer sur les mises à jour du code.

```bash
project/
│
├─ config/
│   ├─ vehicle.yaml
│   ├─ environment.yaml
│   └─ simulation.yaml
│
├─ data/
│   └─ torque_map.csv
│
├─ src/
│   ├─ __init__.py
│   │
│   ├─ engine/
│   │   ├─ __init__.py
│   │   ├─ torque_map.py
│   │   ├─ engine_model.py
│   │   └─ transmission.py
│   │
│   ├─ physics/
│   │   ├─ __init__.py
│   │   ├─ aero.py
│   │   ├─ rolling.py
│   │   ├─ slope.py
│   │   ├─ brake.py
│   │   └─ longitudinal_dynamics.py
│   │
│   ├─ vehicle/
│   │   ├─ __init__.py
│   │   ├─ parameters.py
│   │   └─ vehicle_state.py
│   │
│   ├─ simulation/
│   │   ├─ __init__.py
│   │   ├─ integrator.py
│   │   ├─ controller.py
│   │   └─ runner.py
│   │
│   └─ utils/
│       ├─ __init__.py
│       ├─ loader.py
│       └─ math_utils.py
│
├─ tests/
│   ├─ test_engine.py
│   ├─ test_physics.py
│   └─ test_simulation.py
│
├─ notebooks/
│   └─ exploration.ipynb
│
├─ main.py
│
└─ requirements.txt


```
