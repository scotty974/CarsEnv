import math

import numpy as np

######################## Calcul régime moteur
torque_map_rpm = [
    (800, 420),  # Couple au ralenti
    (1500, 480),  # Montée progressive
    (2500, 520),  # Zone intermédiaire
    (3500, 542),  # Couple maxi à ~4600 rpm
    (4600, 556),  # 556 Nm (couple max officiel)
    (5500, 540),  # Début de chute
    (6500, 500),  # Descente
    (7500, 420),  # Zone rouge
]


def calcul_engine_speed(v, r_roue, trans):
    return (v / r_roue) * trans


def calcul_engine_power(c, omega_engine):
    return c * omega_engine


def make_interpolated_function(rpm):
    rpms = np.array([p for p, t in torque_map_rpm], dtype=float)
    torques = np.array([t for p, t in torque_map_rpm], dtype=float)
    return float(np.interp(rpm, rpms, torques))


######################## Calcul des forces
# paramètres
# p = densité de l'aire
# s = surface frontale de l'objet
# v = vitesse de l'objet
def calcul_aero_drag(p, c, s, v):
    return 0.5 * p * s * c * v**2


# paramètres
# c = coefficient de roulement
# m = masse de l'objet
# g = intensité de la gravité
def calcul_roll(c, m, g):
    return c * m * g


def calcul_pente(mass, g, angle):
    return mass * g * math.sin(angle)


def result_force(forces):
    return np.sum(forces, axis=0)


########################  Calcul accélération et mettre à jour les vitesses
def calcul_acceleration(mass, force):
    return force / mass


def calcul_vitesse(v, acceleration, dt):
    return v + acceleration * dt


def calcul_position(x, v, dt):
    return x + v * dt


######################## Calcul puissance moteur
def calcul_motor_power(motor_force, v):
    return motor_force * v


# on passe ici la puissance moteur calculée au dessus
def calcul_motor_torque(motor_force, trans):
    return motor_force * trans


def calcul_consommation(motor_power, bsfc):
    return motor_power * bsfc / 3600
