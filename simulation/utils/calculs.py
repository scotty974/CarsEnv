import math

import numpy as np


#  Calcul des forces
# paramètres
# p = densité de l'aire
# s = surface frontale de l'objet
# v = vitesse de l'objet
def calcul_aero_drag(p, c, s, v):
    f = 0.5 * p * s * c * v**2
    return f


# paramètres
# c = coefficient de roulement
# m = masse de l'objet
# g = intensité de la gravité
def calcul_roll(c, m, g):
    f = c * m * g
    return f


def calcul_pente(mass, g, angle):
    f = mass * g * math.sin(angle)
    return f


def result_force(forces):
    f = np.sum(forces, axis=0)
    return f


#  Calcul accélération et mettre à jour les vitesses
def calcul_acceleration(mass, force):
    a = force / mass
    return a


def calcul_vitesse(v, acceleration, dt):
    v = v + acceleration * dt
    return v


def calcul_position(x, v, dt):
    x = x + v * dt
    return x


# Calcul puissance moteur


def calcul_motor_power(motor_force, v):
    p = motor_force * v
    return p


# on passe ici la puissance moteur calculée au dessus
def calcul_motor_torque(motor_force, trans):
    p = motor_force * trans
    return p


def calcul_consommation(motor_power, bsfc):
    consommation = motor_power * bsfc / 3600
    return consommation
